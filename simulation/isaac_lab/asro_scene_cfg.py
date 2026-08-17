"""InteractiveSceneCfg for ASRo — the parallelized scene definition.

Structural template copied from Isaac Lab's own tutorial
(scripts/tutorials/02_scene/create_scene.py). This is the piece that's
actually different from anything in ros2_asro.py: {ENV_REGEX_NS} means
Isaac Lab spawns num_envs independent copies of this whole scene
side by side, each with its own instance of the robot, all driven by the
same policy in parallel — this is where the 172k-247k steps/sec we saw
training Cartpole actually comes from.
"""

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils.configclass import configclass

from asro_cfg import ASRO_CFG


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
