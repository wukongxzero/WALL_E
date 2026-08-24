"""InteractiveSceneCfg for ASRo — the parallelized scene definition.

Structural template copied from Isaac Lab's own tutorial
(scripts/tutorials/02_scene/create_scene.py). This is the piece that's
actually different from anything in ros2_asro.py: {ENV_REGEX_NS} means
Isaac Lab spawns num_envs independent copies of this whole scene
side by side, each with its own instance of the robot, all driven by the
same policy in parallel — this is where the 172k-247k steps/sec we saw
training Cartpole actually comes from.

Obstacles are static (no RigidBodyAPI) — visual/observational stand-ins for
what a real YOLO detector would report, not physical hazards yet (see
TerminationsCfg.time_out-only note in asro_navigation_env_cfg.py: no
contact sensor exists yet, so the robot can't actually collide with these).
Positions sit inside the pose_command range (pos_x/pos_y ±3.0m) so the
policy has a reason to notice them while navigating to a goal.
"""

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils.configclass import configclass

from asro_cfg import ASRO_CFG

# (scene key, class id) — class id is the label OBSTACLE_DETECTIONS in
# asro_mdp.py one-hot-encodes. Keep this list and the scene entries below in
# sync: every name here must exist as an AssetBaseCfg field on AsroSceneCfg.
OBSTACLE_REGISTRY = [
    ("obstacle_box_0", 0),
    ("obstacle_box_1", 0),
    ("obstacle_cone_0", 1),
    ("obstacle_cone_1", 1),
]


@configclass
class AsroSceneCfg(InteractiveSceneCfg):
    """Configuration for a scene with one ASRo per parallel environment."""

    # ground plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # robot — one instance per parallel env, {ENV_REGEX_NS} is the per-env prim path
    asro = ASRO_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # obstacles — class 0 ("box"), fixed spawn positions per env, same layout
    # every episode for now (randomizing spawn is a later step, not needed to
    # get the observation term working first)
    obstacle_box_0 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/ObstacleBox0",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(1.5, 1.0, 0.25)),
        spawn=sim_utils.CuboidCfg(
            size=(0.5, 0.5, 0.5),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.2, 0.2)),
        ),
    )
    obstacle_box_1 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/ObstacleBox1",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(-1.5, -2.0, 0.25)),
        spawn=sim_utils.CuboidCfg(
            size=(0.5, 0.5, 0.5),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.2, 0.2)),
        ),
    )

    # obstacles — class 1 ("cone")
    obstacle_cone_0 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/ObstacleCone0",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(-2.0, 2.0, 0.25)),
        spawn=sim_utils.ConeCfg(
            radius=0.25,
            height=0.5,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.6, 0.1)),
        ),
    )
    obstacle_cone_1 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/ObstacleCone1",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(2.0, -1.0, 0.25)),
        spawn=sim_utils.ConeCfg(
            radius=0.25,
            height=0.5,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.6, 0.1)),
        ),
    )
