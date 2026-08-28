"""Isaac Lab ArticulationCfg for ASRo (6-wheel differential-drive rover).

Structural template copied from Isaac Lab's own CARTPOLE_CFG
(isaaclab_assets/robots/cartpole.py) — spawn -> init_state -> actuators is
the standard shape every ArticulationCfg follows.

Joint layout confirmed directly from the imported USD (not guessed):
  defaultPrim:        /blue_shark_land_vehicle
  ArticulationRoot:   /blue_shark_land_vehicle/Geometry/base_footprint
  Wheel joints:       /blue_shark_land_vehicle/Physics/joint_wheel_{fl,fr,ml,mr,rl,rr}

Actuator values (damping=1e4, effort_limit=50.0, stiffness=0.0 for pure
velocity control) are not arbitrary — they're the exact values already
validated driving ASRo correctly in ros2_asro.py's raw DriveAPI setup
(2026-08-13/14 session). Reusing known-working numbers here instead of
re-deriving them from scratch.
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

ASRO_USD_PATH = "/home/wukong/WALL_E/simulation/usd/ASRo_URDF_Simple_6_Wheel_Robot/ASRo_URDF_Simple_6_Wheel_Robot.usda"

WHEEL_JOINT_NAMES = [
    "joint_wheel_fl",
    "joint_wheel_fr",
    "joint_wheel_ml",
    "joint_wheel_mr",
    "joint_wheel_rl",
    "joint_wheel_rr",
]

ASRO_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=ASRO_USD_PATH,
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
        # Same spawn position used throughout yesterday's ros2_asro.py runs —
        # flat ground near WALL-E's spot, identity orientation.
        pos=(0.0, 3.0, 0.3),
        joint_pos={name: 0.0 for name in WHEEL_JOINT_NAMES},
    ),
    actuators={
        "wheel_drives": ImplicitActuatorCfg(
            joint_names_expr=["joint_wheel_.*"],
            effort_limit_sim=30.0,
            stiffness=0.0,
            damping=1e5,
        ),
    },
)
"""Configuration for the ASRo 6-wheel differential-drive rover."""
