"""InteractiveSceneCfg for ASRo -- wires ASRO_CFG into the shared
NavigationSceneCfg (ground, lights, obstacles; see navigation_scene_cfg.py).
"""

from asro_cfg import ASRO_CFG
from isaaclab.assets import ArticulationCfg
from isaaclab.utils.configclass import configclass
from navigation_scene_cfg import NavigationSceneCfg


@configclass
class AsroSceneCfg(NavigationSceneCfg):
    """NavigationSceneCfg with one ASRo per parallel environment."""

    robot: ArticulationCfg = ASRO_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
