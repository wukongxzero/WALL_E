"""Generic InteractiveSceneCfg for wheeled-robot navigation.

Ground, lights, and the fixed obstacle registry -- everything except which
robot occupies the scene. Extracted from asro_scene_cfg.py so WALL-E can
train in the literal same scene (not a duplicate) instead of each robot
getting its own copy of ground/lights/obstacles. Subclass and set `robot`
to a robot-specific ArticulationCfg -- see asro_scene_cfg.py / wall_e_scene_cfg.py.

{ENV_REGEX_NS} means Isaac Lab spawns num_envs independent copies of this
whole scene side by side, each with its own robot instance, all driven by
the same policy in parallel.

Obstacles are static (no RigidBodyAPI) -- visual/observational stand-ins for
what a real YOLO detector would report, not physical hazards yet (no contact
sensor exists on either robot yet, see TerminationsCfg in navigation_env_cfg.py).
Positions sit inside the pose_command range (pos_x/pos_y +/-3.0m) so the
policy has a reason to notice them while navigating to a goal.
"""

from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils.configclass import configclass

# (scene key, class id) -- class id is the label OBSTACLE_DETECTIONS in
# asro_mdp.py one-hot-encodes. Keep this list and the scene entries below in
# sync: every name here must exist as an AssetBaseCfg field on NavigationSceneCfg.
OBSTACLE_REGISTRY = [
    ("obstacle_box_0", 0),
    ("obstacle_box_1", 0),
    ("obstacle_cone_0", 1),
    ("obstacle_cone_1", 1),
]


@configclass
class NavigationSceneCfg(InteractiveSceneCfg):
    """Scene with one robot per parallel environment, navigating around fixed obstacles.

    `robot` is required (MISSING) -- a concrete subclass must set it to a
    robot-specific ArticulationCfg with prim_path="{ENV_REGEX_NS}/Robot".
    """

    robot: ArticulationCfg = MISSING

    # ground plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # obstacles -- class 0 ("box"), fixed spawn positions per env, same layout
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

    # obstacles -- class 1 ("cone")
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
