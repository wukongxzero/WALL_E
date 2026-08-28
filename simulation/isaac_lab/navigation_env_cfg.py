"""Generic ManagerBasedRLEnvCfg for wheeled-robot navigation -- drive to a
randomly commanded 2D goal.

Extracted from asro_navigation_env_cfg.py so ASRo and WALL-E can train in
the literal same task (not two copies of the same logic): everything here
is robot-agnostic -- it only ever refers to the scene entity by the generic
name "robot", never a specific robot's name. A concrete robot's env cfg
(asro_navigation_env_cfg.py, wall_e_navigation_env_cfg.py) subclasses
NavigationEnvCfg and supplies just `scene` and `actions`, both of which are
the only genuinely robot-specific pieces (which ArticulationCfg, which
joint names drive it).

Single-level task, unlike Isaac Lab's own ANYmal navigation example (which
needs a pre-trained low-level walking policy underneath a navigation
policy -- legged robots need that hierarchy, wheeled ones don't). Both
ASRo and WALL-E's policies output wheel/track velocities directly.

Reward/command functions (position_command_error_tanh,
heading_command_error_abs, UniformPose2dCommandCfg) are reused directly from
isaaclab_tasks.manager_based.navigation -- they're generic, robot-agnostic
functions that only read from the command manager, not ANYmal-specific.
"""

import math
from dataclasses import MISSING

from isaaclab.envs import ManagerBasedRLEnvCfg, mdp
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils.configclass import configclass
from isaaclab_tasks.manager_based.navigation.mdp.rewards import (
    heading_command_error_abs,
    position_command_error_tanh,
)
from navigation_scene_cfg import NavigationSceneCfg


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, params={"asset_cfg": SceneEntityCfg("robot")})
        # Tilt awareness via gravity projected into the robot's own frame --
        # Isaac Lab's standard way of giving a policy tilt/orientation
        # sense, instead of a physical tilt sensor.
        projected_gravity = ObsTerm(func=mdp.projected_gravity, params={"asset_cfg": SceneEntityCfg("robot")})
        pose_command = ObsTerm(func=mdp.generated_commands, params={"command_name": "pose_command"})

    policy: PolicyCfg = PolicyCfg()


@configclass
class CommandsCfg:
    """Random 2D goal positions for the robot to navigate to."""

    pose_command = mdp.UniformPose2dCommandCfg(
        asset_name="robot",
        simple_heading=False,
        resampling_time_range=(8.0, 8.0),
        debug_vis=True,
        position_success_threshold=0.5,
        ranges=mdp.UniformPose2dCommandCfg.Ranges(pos_x=(-3.0, 3.0), pos_y=(-3.0, 3.0), heading=(-math.pi, math.pi)),
    )


@configclass
class RewardsCfg:
    """Reward terms -- position/heading tracking, reused from Isaac Lab's navigation task."""

    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-400.0)
    position_tracking = RewTerm(
        func=position_command_error_tanh, weight=0.5, params={"std": 2.0, "command_name": "pose_command"}
    )
    position_tracking_fine_grained = RewTerm(
        func=position_command_error_tanh, weight=0.5, params={"std": 0.2, "command_name": "pose_command"}
    )
    orientation_tracking = RewTerm(
        func=heading_command_error_abs, weight=-0.2, params={"command_name": "pose_command"}
    )


@configclass
class TerminationsCfg:
    """Timeout only for this first pass -- no contact sensor set up on either robot yet."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class EventCfg:
    """Randomize spawn position/heading each episode reset."""

    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (-0.0, 0.0), "y": (-0.0, 0.0), "z": (-0.0, 0.0),
                "roll": (-0.0, 0.0), "pitch": (-0.0, 0.0), "yaw": (-0.0, 0.0),
            },
        },
    )


@configclass
class NavigationEnvCfg(ManagerBasedRLEnvCfg):
    """Robot navigates to a randomly commanded 2D goal position.

    Abstract-ish base: `scene` and `actions` are MISSING and must be set by
    a concrete subclass to the robot-specific scene cfg (with `robot` filled
    in) and an ActionsCfg wired to that robot's actual joint names.
    """

    scene: NavigationSceneCfg = MISSING
    actions: object = MISSING
    observations: ObservationsCfg = ObservationsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        self.decimation = 4
        self.sim.dt = 1.0 / 60.0
        self.sim.render_interval = self.decimation
        self.episode_length_s = self.commands.pose_command.resampling_time_range[1]
