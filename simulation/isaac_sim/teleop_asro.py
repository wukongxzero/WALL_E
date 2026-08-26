"""
ASRo 6-wheel robot — keyboard teleop via real wheel-joint torque.

Controls: W/Up = forward, S/Down = backward, A/Left = turn left, D/Right =
turn right. Held keys accumulate/release a [linear, angular] command vector
(same event-driven pattern as Isaac Sim's own Go2 keyboard example —
isaacsim.robot.policy.examples/.../go2/go2_example.py), converted each
physics step into per-side wheel target velocities.

Run with: ~/isaac-sim/python.sh ~/WALL_E/isaac_sim/teleop_asro.py
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb.input
import isaacsim.core.experimental.utils.app as app_utils
import numpy as np
import omni.appwindow
import omni.usd
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.experimental.utils.stage import open_stage
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

SCENE_USD = "/home/wukong/WALL_E/simulation/usd/wall_e_scene.usd"
ASRO_USD = "/home/wukong/WALL_E/simulation/usd/ASRo_URDF_Simple_6_Wheel_Robot/ASRo_URDF_Simple_6_Wheel_Robot.usda"
ASRO_ROOT_PATH = "/World/asro/Geometry/base_footprint"

LEFT_WHEEL_JOINTS = ["joint_wheel_fl", "joint_wheel_ml", "joint_wheel_rl"]
RIGHT_WHEEL_JOINTS = ["joint_wheel_fr", "joint_wheel_mr", "joint_wheel_rr"]

MAX_LINEAR = 200.0   # deg/s at full forward/back
MAX_ANGULAR = 150.0  # deg/s differential added/subtracted per side when turning

# ── Scene ────────────────────────────────────────────────────────────────────
open_stage(SCENE_USD)
stage = omni.usd.get_context().get_stage()

GroundPlane("/World/GroundPlane")

# The entire /World/Untitled environment (buildings, walls, roads, trees —
# 397 meshes) ships with ZERO PhysicsCollisionAPI applied anywhere. Nothing
# in the city model has ever actually been collidable — that's why the robot
# drives straight through walls. Static triangle meshes don't need convex
# decomposition (that's only required for dynamic/moving bodies), so a plain
# CollisionAPI per mesh is correct and cheap here.
_untitled = stage.GetPrimAtPath("/World/Untitled")
_collider_count = 0
for _prim in Usd.PrimRange(_untitled):
    if _prim.GetTypeName() == "Mesh" and not _prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(_prim)
        _collider_count += 1
print(f"[INFO] Applied CollisionAPI to {_collider_count} environment meshes")

asro_prim = stage.DefinePrim("/World/asro", "Xform")
asro_prim.GetReferences().AddReference(ASRO_USD)
# Back on flat, reliable ground (GroundPlane) near WALL-E's spawn point,
# rather than the bridge platform — that surface settled the robot into a
# tipped orientation (yaw -162°, static ever since) with no explicit upright
# reset to correct it.
UsdGeom.XformCommonAPI(asro_prim).SetTranslate((0.0, 3.0, 0.3))

_friction_mat_prim = stage.DefinePrim("/World/Materials/AsroWheels", "Material")
_friction_mat = UsdPhysics.MaterialAPI.Apply(_friction_mat_prim)
_friction_mat.CreateStaticFrictionAttr().Set(0.9)
_friction_mat.CreateDynamicFrictionAttr().Set(0.8)
_friction_mat.CreateRestitutionAttr().Set(0.0)
for cp in [ASRO_ROOT_PATH, "/World/GroundPlane"]:
    _prim = stage.GetPrimAtPath(cp)
    if _prim and _prim.IsValid():
        UsdShade.MaterialBindingAPI(_prim).Bind(
            UsdShade.Material(_friction_mat_prim), materialPurpose="physics"
        )

# ── Wheel drives — cache DriveAPI objects by side for live updates ────────────
left_drives, right_drives = [], []
for prim in stage.TraverseAll():
    name = prim.GetName()
    if name in LEFT_WHEEL_JOINTS or name in RIGHT_WHEEL_JOINTS:
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTypeAttr("velocity")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(1e4)
        drive.CreateMaxForceAttr(50.0)
        drive.CreateTargetVelocityAttr(0.0)
        (left_drives if name in LEFT_WHEEL_JOINTS else right_drives).append(drive)
        print(f"[INFO] Wheel drive registered → {prim.GetPath()}")

# ── Keyboard teleop — event-driven command accumulation ───────────────────────
_command = np.array([0.0, 0.0])  # [linear, angular], -1..1 range
_KEY_MAP = {
    "W": np.array([1.0, 0.0]), "UP": np.array([1.0, 0.0]),
    "S": np.array([-1.0, 0.0]), "DOWN": np.array([-1.0, 0.0]),
    "A": np.array([0.0, 1.0]), "LEFT": np.array([0.0, 1.0]),
    "D": np.array([0.0, -1.0]), "RIGHT": np.array([0.0, -1.0]),
}

def _on_keyboard_event(event, *args, **kwargs):
    global _command
    if event.input.name in _KEY_MAP:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            _command = _command + _KEY_MAP[event.input.name]
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            _command = _command - _KEY_MAP[event.input.name]
    return True

_appwindow = omni.appwindow.get_default_app_window()
_input = carb.input.acquire_input_interface()
_keyboard = _appwindow.get_keyboard()
_sub_keyboard = _input.subscribe_to_keyboard_events(_keyboard, _on_keyboard_event)

# ── Simulate ──────────────────────────────────────────────────────────────────
SimulationManager.set_physics_dt(1.0 / 60.0)
app_utils.play()

for _ in range(10):
    SimulationManager.step()
    simulation_app.update()

root = Articulation(ASRO_ROOT_PATH)

# Explicit upright orientation reset — nothing was forcing this before, so
# the robot settled at whatever orientation the raw reference + gravity
# contact produced (this is what caused the tipped, stuck state).
root.set_world_poses(
    positions=np.array([[0.0, 3.0, 0.3]]),
    orientations=np.array([[1.0, 0.0, 0.0, 0.0]])  # w,x,y,z identity — upright
)
root.set_velocities(
    linear_velocities=np.array([[0.0, 0.0, 0.0]]),
    angular_velocities=np.array([[0.0, 0.0, 0.0]])
)

_chase_cam_prim = stage.DefinePrim("/World/AsroChaseCam", "Camera")
_chase_up = Gf.Vec3d(0.0, 0.0, 1.0)
from omni.kit.viewport.utility import get_active_viewport

_viewport = get_active_viewport()
if _viewport is not None:
    _viewport.camera_path = _chase_cam_prim.GetPath()

for _ in range(30):
    SimulationManager.step()
    simulation_app.update()

print("[INFO] ASRo teleop ready — W/A/S/D or arrow keys to drive")

for i in range(1_000_000):
    pos, orient = root.get_world_poses()
    pos_np = pos.numpy()[0] if hasattr(pos, "numpy") else np.array(pos)[0]
    q = orient.numpy()[0] if hasattr(orient, "numpy") else np.array(orient)[0]
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    if not root.is_physics_tensor_entity_valid():
        app_utils.play()
        simulation_app.update()

    # Command -> per-side wheel target velocity (deg/s)
    linear_cmd = float(np.clip(_command[0], -1.0, 1.0)) * MAX_LINEAR
    angular_cmd = float(np.clip(_command[1], -1.0, 1.0)) * MAX_ANGULAR
    left_vel = linear_cmd - angular_cmd
    right_vel = linear_cmd + angular_cmd
    for d in left_drives:
        d.GetTargetVelocityAttr().Set(left_vel)
    for d in right_drives:
        d.GetTargetVelocityAttr().Set(right_vel)

    # Chase camera — world eye/center recomputed from actual pose each frame
    _cos_y, _sin_y = np.cos(yaw), np.sin(yaw)
    _local_eye, _local_center = np.array([-1.5, 0.0]), np.array([1.0, 0.0])
    _world_eye_xy = pos_np[:2] + np.array([
        _local_eye[0] * _cos_y - _local_eye[1] * _sin_y,
        _local_eye[0] * _sin_y + _local_eye[1] * _cos_y,
    ])
    _world_center_xy = pos_np[:2] + np.array([
        _local_center[0] * _cos_y - _local_center[1] * _sin_y,
        _local_center[0] * _sin_y + _local_center[1] * _cos_y,
    ])
    _eye = Gf.Vec3d(float(_world_eye_xy[0]), float(_world_eye_xy[1]), float(pos_np[2]) + 1.2)
    _center = Gf.Vec3d(float(_world_center_xy[0]), float(_world_center_xy[1]), float(pos_np[2]) + 0.3)
    _view_matrix = Gf.Matrix4d().SetLookAt(_eye, _center, _chase_up)
    _chase_xform = UsdGeom.Xformable(_chase_cam_prim)
    _chase_xform.ClearXformOpOrder()
    _chase_xform.AddTransformOp().Set(_view_matrix.GetInverse())

    SimulationManager.step()
    RenderingManager.render()
    simulation_app.update()

    if i % 100 == 0:
        print(f"[INFO] Step {i:6d} | x={pos_np[0]:.3f} y={pos_np[1]:.3f} yaw={np.degrees(yaw):.1f}° cmd={_command}")

_input.unsubscribe_to_keyboard_events(_keyboard, _sub_keyboard)
app_utils.stop()
simulation_app.close()
