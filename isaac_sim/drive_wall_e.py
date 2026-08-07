"""
WALL-E Isaac Sim 6.0 — differential drive via direct body velocity.

Differential drive kinematics (same as Gazebo plugin):
  v_linear = r * (wL + wR) / 2
  omega    = r * (wR - wL) / L

Joint DriveAPI traction doesn't work because the URDF track cylinder
rpy is not correctly preserved in the USD export — fix later when
setting up Isaac Lab. This controller is equivalent to what Gazebo
used and is sufficient for demo + RL obs collection.

Run with: ~/isaac-sim/python.sh ~/WALL_E/isaac_sim/drive_wall_e.py
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.utils.stage import open_stage
from isaacsim.core.experimental.objects import DistantLight, GroundPlane
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.experimental.prims import RigidPrim, Articulation
import omni.usd
import numpy as np
from pxr import UsdPhysics, PhysxSchema, Gf

WALL_E_USD   = "/home/wukong/WALL_E/usd/wall_e_scene.usd"
WHEEL_RADIUS = 0.08   # m  (URDF cylinder radius)
WHEEL_BASE   = 0.36   # m  (URDF wheel_separation)
LEFT_VEL     = 5.0    # rad/s
RIGHT_VEL    = 5.0    # rad/s

# Differential drive → body twist
V_LIN = WHEEL_RADIUS * (LEFT_VEL + RIGHT_VEL) / 2     # 0.4 m/s forward
OMEGA = WHEEL_RADIUS * (RIGHT_VEL - LEFT_VEL) / WHEEL_BASE  # 0 rad/s (straight)

TARGET_YAW   = 0.0    # rad — heading to maintain
K_YAW        = 10.0   # P-gain for yaw correction (needs to dominate ~0.042 rad/s drift)

# ── Scene ────────────────────────────────────────────────────────────────────
# Open wall_e_scene.usd directly — the actual combined robot+environment
# scene (/World/wall_e = robot, /World/Untitled = tree/rail environment,
# hundreds of static colliders). Previously this script created a new empty
# stage and referenced in just the robot, which is why it visually looked
# like a different, emptier scene than what open_scene.py showed — same
# robot asset, but no environment, own ground plane/lighting from scratch.
# Loading the file directly instead so what runs matches what's inspected.
open_stage(WALL_E_USD)
stage = omni.usd.get_context().get_stage()

# Physics scene already exists in the file (/PhysicsScene) — just make sure
# gravity is set explicitly rather than relying on it being already-correct.
_scene_prim = stage.DefinePrim("/PhysicsScene", "PhysicsScene")
UsdPhysics.Scene(_scene_prim).CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
UsdPhysics.Scene(_scene_prim).CreateGravityMagnitudeAttr().Set(9.81)

# Defensive ground plane — the file's own /World/Untitled environment may or
# may not include an explicit ground collider; adding one is harmless even
# if redundant with existing terrain.
GroundPlane("/World/GroundPlane")

# ── Friction — physics material bound to track colliders + ground ─────────────
# Friction lives on a PhysicsMaterial, not the rigid body itself. PhysX combines
# both contacting surfaces' materials (default: average), so put it on both the
# track colliders and the ground for it to actually matter.
from pxr import UsdShade
_friction_mat_prim = stage.DefinePrim("/World/Materials/TrackRubber", "Material")
_friction_mat = UsdPhysics.MaterialAPI.Apply(_friction_mat_prim)
_friction_mat.CreateStaticFrictionAttr().Set(0.9)
_friction_mat.CreateDynamicFrictionAttr().Set(0.8)
_friction_mat.CreateRestitutionAttr().Set(0.0)  # no bounce

TRACK_COLLIDER_PATHS = [
    "/World/wall_e/Geometry/base_footprint/base_link/left_track/cylinder_1",
    "/World/wall_e/Geometry/base_footprint/base_link/right_track/cylinder_1",
    "/World/GroundPlane",
]
for cp in TRACK_COLLIDER_PATHS:
    _prim = stage.GetPrimAtPath(cp)
    if _prim and _prim.IsValid():
        UsdShade.MaterialBindingAPI(_prim).Bind(
            UsdShade.Material(_friction_mat_prim), materialPurpose="physics"
        )
        print(f"[INFO] Friction material bound → {cp}")
    else:
        print(f"[WARN] Friction target not found (skipped): {cp}")

# ── DriveAPI — spin the tracks visually in sync with body speed ───────────────
# Body speed = ~0.55 m/s effective (measured), ω_track = v / r
TRACK_VEL_DEG = (V_LIN / WHEEL_RADIUS) * 180.0 / np.pi   # ~286 deg/s
TRACK_JOINTS  = ["left_track_joint", "right_track_joint"]
for prim in stage.TraverseAll():
    if prim.GetName() in TRACK_JOINTS:
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTypeAttr("velocity")
        drive.CreateStiffnessAttr(0.0)
        # damping=1e3, maxForce=1e5 (previous values) were strong enough to
        # physically dominate the body-velocity control — this drive is only
        # meant to spin the tracks cosmetically, not drive the robot (the
        # cylinder's rpy isn't preserved in USD export, so this axis is wrong
        # anyway; see module docstring). Small values so it can't fight the
        # real locomotion control on base_link.
        drive.CreateDampingAttr(10.0)
        drive.CreateMaxForceAttr(5.0)
        drive.CreateTargetVelocityAttr(TRACK_VEL_DEG)
        print(f"[INFO] Track spin {TRACK_VEL_DEG:.1f} deg/s → {prim.GetPath()}")

# ── Simulate ──────────────────────────────────────────────────────────────────
SimulationManager.set_physics_dt(1.0 / 60.0)
app_utils.play()

for _ in range(10):
    SimulationManager.step()
    simulation_app.update()

# All structural mutations (rigid-body enable/disable, depenetration attrs)
# must happen BEFORE constructing Articulation() below — it builds a cached
# physics tensor view at construction time, and mutating sibling rigid-body
# state afterward invalidates/corrupts that view (seen directly: "prim ...
# was deleted while being used by a shape in a tensor view class").

# Cap base_link's depenetration velocity — otherwise any spawn-time collider
# overlap resolves in a single frame at whatever speed clears it (the
# ~140m/~500m launch bug seen earlier). 3 m/s clears a few cm of overlap gently.
_base_link_prim = stage.GetPrimAtPath("/World/wall_e/Geometry/base_footprint/base_link")
PhysxSchema.PhysxRigidBodyAPI.Apply(_base_link_prim).CreateMaxDepenetrationVelocityAttr().Set(3.0)

# Sensor links (camera/imu) report near-zero/invalid inertia ({1,1,1} +
# negative mass) since they're just optical-frame markers with no real mesh
# mass. Previously disabled RigidBodyAPI entirely to silence the warning —
# but they're still joint-connected to base_link, and disabling one side of
# a joint breaks the constraint (this is exactly what the recurring
# "PxJoint::setActors: at least one actor must be non-static" error meant —
# it made the sensors detach and fall, not just print a warning). Fix: give
# them small real mass instead, so the joint stays valid and they stay
# rigidly attached.
from pxr import UsdPhysics as UsdPhy
sensor_paths = [
    "/World/wall_e/Geometry/base_footprint/base_link/camera_link/camera_color_optical_frame",
    "/World/wall_e/Geometry/base_footprint/base_link/camera_link/camera_depth_optical_frame",
    "/World/wall_e/Geometry/base_footprint/base_link/imu_link",
]
for sp in sensor_paths:
    prim = stage.GetPrimAtPath(sp)
    if prim and prim.IsValid():
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr().Set(0.01)  # kg — small but valid, real sensors are light

# Articulation (not RigidPrim) — base_link is an ArticulationRoot with 8
# joint-connected child bodies (tracks, supports, camera, imu). Setting
# velocity via RigidPrim treats it as a standalone rigid body and fights the
# articulation solver's own joint-constraint handling for its children —
# that mismatch was the actual cause of the deterministic explosion seen
# earlier (same resting state every run, regardless of file/force tuning).
# Articulation.set_velocities() goes through the proper articulation root
# velocity view instead, which coordinates correctly with the joints.
# Constructed AFTER all structural mutations above so its cached view is built
# against the final physics state, not invalidated by later changes.
root = Articulation("/World/wall_e/Geometry/base_footprint/base_link")

# Fix spawn position — USD has a baked-in offset, reset to origin
root.set_world_poses(
    positions=np.array([[0.0, 0.0, 0.1]]),
    orientations=np.array([[1.0, 0.0, 0.0, 0.0]])  # w,x,y,z identity
)
root.set_velocities(
    linear_velocities=np.array([[0.0, 0.0, 0.0]]),
    angular_velocities=np.array([[0.0, 0.0, 0.0]])
)

# Top-down viewport camera — removes ambiguity about "sideways vs forward."
# From directly overhead, +X motion reads as unambiguous translation along
# one screen axis regardless of the default camera's arbitrary starting angle.
from omni.kit.viewport.utility import get_active_viewport, camera_state
_viewport = get_active_viewport()
if _viewport is not None:
    _cam_state = camera_state.ViewportCameraState(_viewport.camera_path, _viewport)
    _cam_state.set_target_world(Gf.Vec3d(0.0, 0.0, 0.0), True)
    _cam_state.set_position_world(Gf.Vec3d(0.0, -1.5, 2.5), True)  # close overhead, slightly behind

# Let any residual spawn overlap resolve gently (capped at 3 m/s now) before
# the control loop starts commanding velocity on top of it.
for _ in range(30):
    SimulationManager.step()
    simulation_app.update()

settle_pos, _ = root.get_world_poses()
settle_pos_np = settle_pos.numpy()[0] if hasattr(settle_pos, "numpy") else np.array(settle_pos)[0]
print(f"[INFO] Post-settle position: x={settle_pos_np[0]:.3f} y={settle_pos_np[1]:.3f} z={settle_pos_np[2]:.3f}")

print(f"[INFO] WALL-E ready — driving at {V_LIN:.2f} m/s straight")

for i in range(1_000_000):
    # ── Get current heading (yaw) from quaternion ─────────────────────────
    pos, orient = root.get_world_poses()
    pos_np = pos.numpy()[0] if hasattr(pos, "numpy") else np.array(pos)[0]
    q = orient.numpy()[0] if hasattr(orient, "numpy") else np.array(orient)[0]

    # Isaac Sim quaternion order: [w, x, y, z]
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    # Rotate body-frame velocity to world frame
    vx = V_LIN * np.cos(yaw)
    vy = V_LIN * np.sin(yaw)

    # Yaw P-controller — actively correct drift toward TARGET_YAW
    yaw_err = TARGET_YAW - yaw
    yaw_err = (yaw_err + np.pi) % (2 * np.pi) - np.pi  # wrap to [-π, π]
    omega_cmd = OMEGA + K_YAW * yaw_err

    # Physics tensor view can drop out of validity between steps (assertion
    # explicitly says "Play the simulation/timeline to re-initialize it") —
    # re-arm defensively rather than crash on it.
    if not root.is_physics_tensor_entity_valid():
        app_utils.play()
        simulation_app.update()

    root.set_velocities(
        linear_velocities=np.array([[vx, vy, 0.0]]),
        angular_velocities=np.array([[0.0, 0.0, omega_cmd]])
    )

    SimulationManager.step()
    RenderingManager.render()
    simulation_app.update()

    if i % 100 == 0:
        print(f"[INFO] Step {i:6d} | x={pos_np[0]:.3f} y={pos_np[1]:.3f} yaw={np.degrees(yaw):.1f}° ω_cmd={omega_cmd:.3f}")

app_utils.stop()
simulation_app.close()
