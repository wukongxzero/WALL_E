"""InteractiveSceneCfg for WALL-E -- wires WALL_E_CFG into the shared
NavigationSceneCfg (ground, lights, obstacles; see navigation_scene_cfg.py).
"""

from isaaclab.assets import ArticulationCfg
from isaaclab.utils.configclass import configclass
from navigation_scene_cfg import NavigationSceneCfg
from wall_e_cfg import WALL_E_CFG


@configclass
class WallESceneCfg(NavigationSceneCfg):
    """NavigationSceneCfg with one WALL-E per parallel environment."""

    robot: ArticulationCfg = WALL_E_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
