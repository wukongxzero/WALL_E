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

from isaaclab.assets import Articulation
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import subtract_frame_transforms

from asro_scene_cfg import OBSTACLE_REGISTRY


def fake_yolo_detections(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("asro"),
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
       world position the same way (env.scene[name].data.root_pos_w.torch)
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
    raise NotImplementedError
