"""
ASRo 6-wheel robot — differential drive via real wheel-joint torque.

Unlike WALL-E (URDF->USD conversion lost the track cylinder orientation,
so joint drive never worked — see drive_wall_e.py docstring), ASRo went
through Isaac's proper URDFImporter Python API and came out with correct
PhysicsRevoluteJoint axes (Y, matching the URDF). This means it can
actually be driven through wheel-joint DriveAPI, not a body-velocity
workaround.

Run with: ~/isaac-sim/python.sh ~/WALL_E/isaac_sim/drive_asro.py
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import isaacsim.core.experimental.utils.app as app_utils
import numpy as np
import omni.usd
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.experimental.utils.stage import open_stage
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, UsdGeom, UsdPhysics, UsdShade

SCENE_USD = "/home/wukong/WALL_E/simulation/usd/wall_e_scene.usd"
ASRO_USD = "/home/wukong/WALL_E/simulation/usd/ASRo_URDF_Simple_6_Wheel_Robot/ASRo_URDF_Simple_6_Wheel_Robot.usda"
ASRO_ROOT_PATH = "/World/asro/Geometry/base_footprint"

WHEEL_TARGET_VEL_DEG = 200.0  # deg/s — real wheel-joint drive this time, not cosmetic
WHEEL_JOINTS = [
    "joint_wheel_fl", "joint_wheel_fr",
    "joint_wheel_ml", "joint_wheel_mr",
    "joint_wheel_rl", "joint_wheel_rr",
]

# ── Scene ────────────────────────────────────────────────────────────────────
open_stage(SCENE_USD)
stage = omni.usd.get_context().get_stage()

GroundPlane("/World/GroundPlane")

asro_prim = stage.DefinePrim("/World/asro", "Xform")
asro_prim.GetReferences().AddReference(ASRO_USD)
UsdGeom.XformCommonAPI(asro_prim).SetTranslate((0.0, 3.0, 0.3))

# Friction on wheels + ground
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

# ── Real wheel-joint drive ──────────────────────────────────────────────────
for prim in stage.TraverseAll():
    if prim.GetName() in WHEEL_JOINTS:
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTypeAttr("velocity")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(1e4)
        drive.CreateMaxForceAttr(50.0)
        drive.CreateTargetVelocityAttr(WHEEL_TARGET_VEL_DEG)
        print(f"[INFO] Wheel drive {WHEEL_TARGET_VEL_DEG:.1f} deg/s → {prim.GetPath()}")

# ── Simulate ──────────────────────────────────────────────────────────────────
SimulationManager.set_physics_dt(1.0 / 60.0)
app_utils.play()

for _ in range(10):
    SimulationManager.step()
    simulation_app.update()

root = Articulation(ASRO_ROOT_PATH)

# Chase camera — recomputed every frame from actual pose (see drive_wall_e.py
# for why this approach beats a fixed one-time position or USD-parented cam).
_chase_cam_prim = stage.DefinePrim("/World/AsroChaseCam", "Camera")
_chase_up = Gf.Vec3d(0.0, 0.0, 1.0)
from omni.kit.viewport.utility import get_active_viewport

_viewport = get_active_viewport()
if _viewport is not None:
    _viewport.camera_path = _chase_cam_prim.GetPath()

for _ in range(30):
    SimulationManager.step()
    simulation_app.update()

settle_pos, _ = root.get_world_poses()
settle_pos_np = settle_pos.numpy()[0] if hasattr(settle_pos, "numpy") else np.array(settle_pos)[0]
print(f"[INFO] Post-settle position: x={settle_pos_np[0]:.3f} y={settle_pos_np[1]:.3f} z={settle_pos_np[2]:.3f}")
print("[INFO] ASRo ready — driving via real wheel-joint torque")

for i in range(1_000_000):
    pos, orient = root.get_world_poses()
    pos_np = pos.numpy()[0] if hasattr(pos, "numpy") else np.array(pos)[0]
    q = orient.numpy()[0] if hasattr(orient, "numpy") else np.array(orient)[0]
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    if not root.is_physics_tensor_entity_valid():
        app_utils.play()
        simulation_app.update()

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
        print(f"[INFO] Step {i:6d} | x={pos_np[0]:.3f} y={pos_np[1]:.3f} yaw={np.degrees(yaw):.1f}°")

app_utils.stop()
simulation_app.close()
