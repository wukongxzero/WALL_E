"""ManagerBasedRLEnvCfg for ASRo -- wires ASRo's scene and wheel actions into
the shared NavigationEnvCfg (rewards/observations/commands/terminations/events
are identical for every robot on this task; see navigation_env_cfg.py).
"""

from asro_cfg import WHEEL_JOINT_NAMES
from asro_scene_cfg import AsroSceneCfg
from isaaclab.envs import mdp
from isaaclab.utils.configclass import configclass
from navigation_env_cfg import NavigationEnvCfg


@configclass
class ActionsCfg:
    """All 6 wheels driven directly by the policy's velocity output."""

    wheel_vel = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=WHEEL_JOINT_NAMES,
        scale=200.0,  # deg/s-equivalent range validated driving ASRo all week
    )


@configclass
class AsroNavigationEnvCfg(NavigationEnvCfg):
    """ASRo navigates to a randomly commanded 2D goal position."""

    scene: AsroSceneCfg = AsroSceneCfg(num_envs=64, env_spacing=4.0)
    actions: ActionsCfg = ActionsCfg()
