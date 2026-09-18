"""Custom MDP terms for ASRo navigation — not part of isaaclab_tasks, ASRo-specific.

fake_yolo_detections() is the train-time stand-in for yolo_nav_node.py: same
observation *shape* a real detector+depth-backprojection would produce
(nearest-K objects, each as [range, bearing, one_hot(class)]), but read
straight from sim ground truth instead of running YOLO on rendered frames —
YOLO on 64 parallel camera renders every step would tank training throughput.
Swap the body for a real ROS2-fed version at deploy time; the ObsTerm
registration in asro_navigation_env_cfg.py doesn't need to change since the
output shape stays identical.
"""

import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import subtract_frame_transforms
from navigation_scene_cfg import OBSTACLE_REGISTRY

_ROBOT_ENTITY_CFG = SceneEntityCfg("robot")


def fake_yolo_detections(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = _ROBOT_ENTITY_CFG,
    num_classes: int = 2,
    k: int = 3,
) -> torch.Tensor:
    """Nearest-K obstacles in the robot's local frame, YOLO-detection-shaped.

    Per-obstacle feature layout: [range, bearing, one_hot(class)] ->
    (2 + num_classes) floats. Output shape: (num_envs, k * (2 + num_classes)).

    Steps (fill in below):
    1. Get the robot's world pose:
         asset: Articulation = env.scene[asset_cfg.name]
         robot_pos_w = asset.data.root_pos_w.torch   # (num_envs, 3)
         robot_quat_w = asset.data.root_quat_w.torch  # (num_envs, 4), xyzw

    2. For each (name, class_id) in OBSTACLE_REGISTRY, get that obstacle's
       world position: env.scene.env_origins + the obstacle's init_state.pos
       (static AssetBaseCfg prims have no .data, and their FrameView only
       covers env 0, so it can't be used per-env)
       and use subtract_frame_transforms(robot_pos_w, robot_quat_w, obs_pos_w)
       to get its position in the robot's local frame. rel_pos[:, :2] is
       local (x, y) — forward/left, same convention the wheel controller
       already uses.

    3. From local (x, y): range = norm, bearing = atan2(y, x). Build the
       (2 + num_classes) feature vector per obstacle, one-hot the class_id
       into num_classes slots.

    4. Stack all len(OBSTACLE_REGISTRY) obstacles into (num_envs, N, feat_dim),
       sort by range ascending (torch.sort / torch.topk on the range column,
       gather the rest), take the nearest k. OBSTACLE_REGISTRY currently has
       4 entries and k defaults to 3, so there's always at least one dropped
       — no padding/masking needed yet. (Add a zero-pad + "valid" flag slot
       later if OBSTACLE_REGISTRY ever shrinks below k, or if you want
       range-gating to drop far-away obstacles instead of just top-k.)

    5. Flatten (num_envs, k, feat_dim) -> (num_envs, k * feat_dim) since
       ObsTerm outputs must be a flat (num_envs, obs_dim) tensor.
    """

    asset = env.scene[asset_cfg.name]

    robot_pos_w = asset.data.root_pos_w.torch
    robot_quat_w = asset.data.root_quat_w.torch
    local_xy_per_obstacle = []

    for name, class_id in OBSTACLE_REGISTRY:
        obs_init_pos = torch.tensor(getattr(env.scene.cfg, name).init_state.pos, device=robot_pos_w.device)
        obs_pos_w = env.scene.env_origins + obs_init_pos  # (num_envs, 3); static obstacles, same layout per env
        rel_pos, _ = subtract_frame_transforms(robot_pos_w, robot_quat_w, obs_pos_w)
        local_xy_per_obstacle.append(rel_pos[:, :2])

    local_xy = torch.stack(local_xy_per_obstacle, dim=1)  # (num_envs, N, 2)
    range_ = torch.norm(local_xy, dim=-1)  # (num_envs, N)
    bearing = torch.atan2(local_xy[..., 1], local_xy[..., 0])  # (num_envs, N)

    class_ids = torch.tensor([cid for _, cid in OBSTACLE_REGISTRY], device=robot_pos_w.device)  # (N,)
    one_hot = torch.nn.functional.one_hot(class_ids, num_classes).float()  # (N, num_classes)
    one_hot = one_hot.unsqueeze(0).expand(env.num_envs, -1, -1)  # (num_envs, N, num_classes)

    features = torch.cat([range_.unsqueeze(-1), bearing.unsqueeze(-1), one_hot], dim=-1)  # (num_envs, N, 2 + num_classes)

    nearest = torch.topk(range_, k, dim=1, largest=False).indices  # (num_envs, k)
    nearest = nearest.unsqueeze(-1).expand(-1, -1, features.shape[-1])  # (num_envs, k, feat_dim)
    features = torch.gather(features, 1, nearest)  # (num_envs, k, feat_dim)

    return features.reshape(env.num_envs, -1)  # (num_envs, k * feat_dim)


def obstacle_proximity_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = _ROBOT_ENTITY_CFG,
    threshold: float = 1.25,
) -> torch.Tensor:
    """0 when the nearest obstacle is farther than `threshold` (center-to-center,
    metres), rising linearly toward 1 as it gets closer. Use with a negative
    RewTerm weight. Contact happens at ~0.75 m center-to-center (measured), so
    threshold=1.25 gives a ~0.5 m warning band before touching.
    """
    robot_xy = env.scene[asset_cfg.name].data.root_pos_w.torch[:, :2]  # (num_envs, 2)
    obstacle_xy = torch.stack(
        [
            env.scene.env_origins[:, :2]
            + torch.tensor(getattr(env.scene.cfg, name).init_state.pos[:2], device=robot_xy.device)
            for name, _ in OBSTACLE_REGISTRY
        ],
        dim=1,
    )  # (num_envs, N, 2)
    nearest = torch.norm(obstacle_xy - robot_xy.unsqueeze(1), dim=-1).min(dim=1).values  # (num_envs,)
    return torch.clamp(threshold - nearest, min=0.0) / threshold
