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
from isaacsim.core.experimental.objects import DistantLight, GroundPlane
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.experimental.prims import RigidPrim
import omni.usd
import numpy as np
from pxr import UsdPhysics

WALL_E_USD   = "/home/wukong/WALL_E/usd/wall_e_v2.usd"
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
stage_utils.create_new_stage()
GroundPlane("/World/GroundPlane")
light = DistantLight("/World/DistantLight")
light.set_intensities(3000)

stage = omni.usd.get_context().get_stage()
wall_e_prim = stage.DefinePrim("/World/wall_e", "Xform")
wall_e_prim.GetReferences().AddReference(WALL_E_USD)

# ── DriveAPI — spin the tracks visually in sync with body speed ───────────────
# Body speed = ~0.55 m/s effective (measured), ω_track = v / r
TRACK_VEL_DEG = (V_LIN / WHEEL_RADIUS) * 180.0 / np.pi   # ~286 deg/s
TRACK_JOINTS  = ["left_track_joint", "right_track_joint"]
for prim in stage.TraverseAll():
    if prim.GetName() in TRACK_JOINTS:
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTypeAttr("velocity")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(1e3)
        drive.CreateMaxForceAttr(1e5)
        drive.CreateTargetVelocityAttr(TRACK_VEL_DEG)
        print(f"[INFO] Track spin {TRACK_VEL_DEG:.1f} deg/s → {prim.GetPath()}")

# ── Simulate ──────────────────────────────────────────────────────────────────
SimulationManager.set_physics_dt(1.0 / 60.0)
app_utils.play()

for _ in range(10):
    SimulationManager.step()
    simulation_app.update()

root = RigidPrim("/World/wall_e/wall_e/Geometry/base_footprint")

# Fix spawn position — USD has a baked-in offset, reset to origin
root.set_world_poses(
    positions=np.array([[0.0, 0.0, 0.1]]),
    orientations=np.array([[1.0, 0.0, 0.0, 0.0]])  # w,x,y,z identity
)
root.set_velocities(
    linear_velocities=np.array([[0.0, 0.0, 0.0]]),
    angular_velocities=np.array([[0.0, 0.0, 0.0]])
)

# Disable rigid body physics on sensor links to prevent inertia blow-up
from pxr import UsdPhysics as UsdPhy
sensor_paths = [
    "/World/wall_e/wall_e/Geometry/base_footprint/base_link/camera_link/camera_color_optical_frame",
    "/World/wall_e/wall_e/Geometry/base_footprint/base_link/camera_link/camera_depth_optical_frame",
    "/World/wall_e/wall_e/Geometry/base_footprint/base_link/imu_link",
]
for sp in sensor_paths:
    prim = stage.GetPrimAtPath(sp)
    if prim and prim.IsValid():
        UsdPhy.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Set(False)

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
