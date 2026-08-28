"""ManagerBasedRLEnvCfg for WALL-E -- wires WALL-E's scene and track actions
into the shared NavigationEnvCfg (rewards/observations/commands/terminations/
events are identical for every robot on this task; see navigation_env_cfg.py).
"""

from isaaclab.envs import mdp
from isaaclab.utils.configclass import configclass
from navigation_env_cfg import NavigationEnvCfg
from wall_e_cfg import TRACK_JOINT_NAMES
from wall_e_scene_cfg import WallESceneCfg


@configclass
class ActionsCfg:
    """Both tracks driven directly by the policy's velocity output."""

    track_vel = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=TRACK_JOINT_NAMES,
        # rad/s -- matches the 5.0 rad/s target validated in test_wall_e_cfg.py
        # (real net movement confirmed under the same drive params used here,
        # PhysicsDriveAPI damping=1e5/effort_limit=1e5 from Physics.usda).
        scale=5.0,
    )


@configclass
class WallENavigationEnvCfg(NavigationEnvCfg):
    """WALL-E navigates to a randomly commanded 2D goal position."""

    scene: WallESceneCfg = WallESceneCfg(num_envs=64, env_spacing=4.0)
    actions: ActionsCfg = ActionsCfg()
