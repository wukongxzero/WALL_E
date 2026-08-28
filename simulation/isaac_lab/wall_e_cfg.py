"""Isaac Lab ArticulationCfg for WALL-E (2-track differential-drive rover).

Structural template copied from asro_cfg.py, which itself follows Isaac
Lab's own CARTPOLE_CFG (isaaclab_assets/robots/cartpole.py) — spawn ->
init_state -> actuators.

Joint layout confirmed from simulation/usd/wall_e.usd/Payload/Physics.usda:
  ArticulationRoot:  /wall_e/Geometry/base_footprint/base_link
  Track joints:      /wall_e/Geometry/base_footprint/base_link/{left,right}_track_joint

base_link (not base_footprint) is the articulation root — base_footprint is
a plain bookkeeping Xform with no PhysicsRigidBodyAPI. Until 2026-08-28 the
robot was also welded to the world via a base_footprint_joint fixed-jointing
base_link to the non-rigid-body reference container it's spawned under;
that joint is gone now, and base_link is a free floating base like any
other mobile-robot articulation root here.

Actuator values (damping=1e5, stiffness=0 for pure velocity control,
maxForce/effort_limit=1e5) are carried over from the PhysicsDriveAPI added
to Physics.usda in the same 2026-08-28 fix that got the tracks actually
driving (verified via direct Articulation.set_joint_velocity_targets and
via the ROS2/OmniGraph chain — see simulation/requirements.txt). Not yet
re-validated inside Isaac Lab's ImplicitActuatorCfg specifically, which is
a different code path than the OmniGraph IsaacArticulationController this
was tested through — treat as a starting point, not a proven-good number
for this framework.
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

WALL_E_USD_PATH = "/home/wukong/WALL_E/simulation/usd/wall_e.usd/wall_e.usda"

TRACK_JOINT_NAMES = [
    "left_track_joint",
    "right_track_joint",
]

WALL_E_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=WALL_E_USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # base_link's own natural resting height above base_footprint/ground
        # (xformOp:translate in Geometry.usda) — same value the removed
        # base_footprint_joint used to provide.
        pos=(0.0, 0.0, 0.2025),
        joint_pos={name: 0.0 for name in TRACK_JOINT_NAMES},
    ),
    actuators={
        "track_drives": ImplicitActuatorCfg(
            joint_names_expr=TRACK_JOINT_NAMES,
            # 1e5 (carried over from the raw USD PhysicsDriveAPI value tuned
            # for the OmniGraph test) let the velocity servo slam into torque
            # far beyond what ~0.5 friction on an 8kg robot can convert to
            # forward force -- tracks just slipped in place. Confirmed via
            # test_navigation_behavior.py: error_pos never closed at 1e5.
            # 30.0 matches ASRO_CFG's effort_limit_sim, which does navigate
            # correctly under the same test.
            effort_limit_sim=30.0,
            stiffness=0.0,
            damping=1e5,
        ),
    },
)
"""Configuration for the WALL-E 2-track differential-drive rover."""
