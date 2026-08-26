"""
ASRo Isaac Sim 6.0 — ROS2 bridge with RealSense D455 RGB-D camera for SLAM.

Publishes:
  /camera/color/image_raw          (sensor_msgs/Image, RGB)
  /camera/color/camera_info        (sensor_msgs/CameraInfo)
  /camera/depth/image_rect_raw     (sensor_msgs/Image, depth)
  /camera/depth/camera_info        (sensor_msgs/CameraInfo)
  /imu                             (sensor_msgs/Imu, 60 Hz)
  /odom                            (nav_msgs/Odometry, 60 Hz)
  /tf                              (odom -> base_footprint, dynamic;
                                     base_footprint -> camera_*_optical_frame, static;
                                     base_footprint -> imu_link, static)
  /clock                           (rosgraph_msgs/Clock, 60 Hz)

Subscribes:
  /cmd_vel                         (geometry_msgs/Twist)

D435 isn't in this Isaac Sim install's bundled sensor registry (only D455/
D457/D555 are) — using D455 as the closest stand-in; same RealSense family,
same RGB + stereo-derived depth design. The asset (rsd455.usd) contains
multiple Camera-typed prims under RSD455/ (color, left/right IR, and a
"Pseudo_Depth" prim that already carries realistic stereo-depth-sensor
postprocessing attributes) — Camera_OmniVision_OV9782_Color and
Camera_Pseudo_Depth are the two actually used here.

Physics attachment goes on the WRAPPER prim (D455), not the individual
camera sub-prims — same lesson learned from the lidar: the sub-prims have
no physics of their own, so a FixedJoint on one of them fights a static
parent that never moves with the robot. The wrapper carries the whole
subtree along via ordinary USD parenting once IT has the FixedJoint.

The IMU is a physics sensor, not an RTX one — no render pipeline involved,
so none of the render-product-attachment issues apply. It reads directly
from the physics engine via the C++ IImuSensor interface, and determines
which rigid body it measures from its OWN parent path (the sensor authoring
class derives "_body_prim_path" from everything but the last path segment),
so placing it directly under ASRO_ROOT_PATH is sufficient — no FixedJoint
needed at all.

Drive is via real wheel-joint torque (see teleop_asro.py docstring — unlike
WALL-E, ASRo's URDF import preserved correct joint axes, so this is genuine
differential-drive wheel control, not a body-velocity workaround).

Run with:
  source /opt/ros/jazzy/setup.bash
  ~/isaac-sim/python.sh ~/WALL_E/isaac_sim/ros2_asro.py
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import isaacsim.core.experimental.utils.app as app_utils
import numpy as np
import omni.graph.core as og
import omni.usd
import usdrt.Sdf
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.experimental.utils.stage import open_stage
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

# ── Enable ROS2 bridge (must happen before rclpy import) ───────────────────────
app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from isaacsim.sensors.experimental.physics import IMU, IMUSensor
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

SCENE_USD = "/home/wukong/WALL_E/simulation/usd/wall_e_scene.usd"
ASRO_USD = "/home/wukong/WALL_E/simulation/usd/ASRo_URDF_Simple_6_Wheel_Robot/ASRo_URDF_Simple_6_Wheel_Robot.usda"
ASRO_ROOT_PATH = "/World/asro/Geometry/base_footprint"

LEFT_WHEEL_JOINTS = ["joint_wheel_fl", "joint_wheel_ml", "joint_wheel_rl"]
RIGHT_WHEEL_JOINTS = ["joint_wheel_fr", "joint_wheel_mr", "joint_wheel_rr"]
MAX_LINEAR_DEG = 200.0
MAX_ANGULAR_DEG = 150.0

CAMERA_PATH = "/World/asro/D455"
CAMERA_HEIGHT = 0.5  # m above base_footprint origin — chassis box top is at 0.45
                      # (base_link sits 0.3 above base_footprint, its own box
                      # extends +-0.15 from there), same mount height as the
                      # earlier lidar mount so it sits on top, not inside it
COLOR_PATH = f"{CAMERA_PATH}/RSD455/Camera_OmniVision_OV9782_Color"
DEPTH_PATH = f"{CAMERA_PATH}/RSD455/Camera_Pseudo_Depth"

IMU_PATH = f"{ASRO_ROOT_PATH}/Imu"

# Standard ROS optical-frame rotation (REP 103/104): rotates from a
# body-style frame (x-forward, y-left, z-up) to the optical convention
# (x-right, y-down, z-forward) that camera_info/image projection assumes.
# Same constant every ROS camera driver (realsense-ros included) publishes
# for <link> -> <link>_optical_frame.
_OPTICAL_ROTATION = (-0.5, 0.5, -0.5, 0.5)  # x, y, z, w

# ── Scene ────────────────────────────────────────────────────────────────────
open_stage(SCENE_USD)
stage = omni.usd.get_context().get_stage()
GroundPlane("/World/GroundPlane")

# Same environment-collision fix as teleop_asro.py — the city model ships with
# zero PhysicsCollisionAPI applied anywhere.
_untitled = stage.GetPrimAtPath("/World/Untitled")
for _prim in Usd.PrimRange(_untitled):
    if _prim.GetTypeName() == "Mesh" and not _prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(_prim)

asro_prim = stage.DefinePrim("/World/asro", "Xform")
asro_prim.GetReferences().AddReference(ASRO_USD)
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

# ── Wheel drives — cache DriveAPI objects by side for live cmd_vel updates ────
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

# ── RealSense D455 — RGB + stereo-derived depth ────────────────────────────────
camera_prim = stage.DefinePrim(CAMERA_PATH, "Xform")
camera_prim.GetReferences().AddReference(
    get_assets_root_path() + "/Isaac/Sensors/RealSense/D455/rsd455.usd"
)
simulation_app.update()

# The rsd455.usd asset mounts its leaf camera prims so the RIG's local -Y is
# the actual optical forward axis and +X is "up" (verified by inspecting the
# leaf cameras' baked xformOp:orient) — not the +X-forward/Z-up convention
# base_footprint uses. With zero rotation here the D455 ended up staring
# sideways off the robot's flank into open space (100% inf depth, uniform
# dome-light-colored RGB — it was rendering nothing but sky/background).
# This quaternion rotates the wrapper so its -Y (forward) aligns with
# base_footprint's +X (the robot's actual drive-forward, confirmed via a
# real cmd_vel test) and its +X (up) aligns with world Z.
_CAMERA_MOUNT_ORIENT = Gf.Quatf(0.5, Gf.Vec3f(-0.5, 0.5, -0.5))  # (w, x, y, z)
UsdGeom.Xformable(camera_prim).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, CAMERA_HEIGHT))
UsdGeom.Xformable(camera_prim).AddOrientOp().Set(_CAMERA_MOUNT_ORIENT)

# Real physics attachment — a rigid body + FixedJoint to the chassis, same
# mechanism WALL-E's URDF-defined sensor links (and the earlier lidar mount)
# use. Applied to the WRAPPER, not the individual color/depth camera prims —
# they have no physics of their own. Must happen before the Articulation()
# tensor view is constructed further down — same structural-mutation-
# ordering rule that fixed the WALL-E camera/imu tensor-view crashes.
UsdPhysics.RigidBodyAPI.Apply(camera_prim)
_cam_mass_api = UsdPhysics.MassAPI.Apply(camera_prim)
_cam_mass_api.CreateMassAttr(0.01)  # tiny real mass — avoids the zero/negative-mass tensor bug

_cam_joint = UsdPhysics.FixedJoint.Define(stage, f"{CAMERA_PATH}/FixedJoint")
_cam_joint.CreateBody0Rel().SetTargets([ASRO_ROOT_PATH])
_cam_joint.CreateBody1Rel().SetTargets([CAMERA_PATH])
_cam_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, CAMERA_HEIGHT))
# Same mount-orientation fix as the Xform ops above — body1 (the camera) has
# no relative rotation of its own (LocalRot1 stays identity), so the joint's
# LocalRot0 alone is what has to carry the wrapper's forward-axis correction.
_cam_joint.CreateLocalRot0Attr().Set(_CAMERA_MOUNT_ORIENT)
_cam_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
_cam_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
print("[INFO] D455 rigidly attached to chassis via FixedJoint")

# ── IMU — physics sensor, no render pipeline, no FixedJoint needed ────────────
# Placed directly under ASRO_ROOT_PATH so its auto-derived body path IS
# base_footprint. Read each frame in the main loop via IMUSensor.get_data(),
# same style as root.get_world_poses() — no OmniGraph needed either.
IMU.create(IMU_PATH, translations=[[0.0, 0.0, 0.0]], orientations=[[1.0, 0.0, 0.0, 0.0]])
imu_sensor = IMUSensor(IMU_PATH)
print("[INFO] IMU created -> /imu")

# ── ROS2 OmniGraph: color + depth -> /camera/... ────────────────────────────────
keys = og.Controller.Keys
ros_camera_graph, _, _, _ = og.Controller.edit(
    {"graph_path": "/World/ROS2_Camera", "evaluator_name": "push",
     "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
    {
        keys.CREATE_NODES: [
            ("OnTick",        "omni.graph.action.OnTick"),
            ("CreateRPColor", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("CreateRPDepth", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("RGBPub",        "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("ColorInfoPub",  "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ("DepthPub",      "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("DepthInfoPub",  "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick",                     "CreateRPColor.inputs:execIn"),
            ("OnTick.outputs:tick",                     "CreateRPDepth.inputs:execIn"),
            ("CreateRPColor.outputs:execOut",           "RGBPub.inputs:execIn"),
            ("CreateRPColor.outputs:execOut",           "ColorInfoPub.inputs:execIn"),
            ("CreateRPColor.outputs:renderProductPath", "RGBPub.inputs:renderProductPath"),
            ("CreateRPColor.outputs:renderProductPath", "ColorInfoPub.inputs:renderProductPath"),
            ("CreateRPDepth.outputs:execOut",           "DepthPub.inputs:execIn"),
            ("CreateRPDepth.outputs:execOut",           "DepthInfoPub.inputs:execIn"),
            ("CreateRPDepth.outputs:renderProductPath", "DepthPub.inputs:renderProductPath"),
            ("CreateRPDepth.outputs:renderProductPath", "DepthInfoPub.inputs:renderProductPath"),
        ],
        keys.SET_VALUES: [
            ("CreateRPColor.inputs:cameraPrim", [usdrt.Sdf.Path(COLOR_PATH)]),
            ("CreateRPDepth.inputs:cameraPrim", [usdrt.Sdf.Path(DEPTH_PATH)]),
            ("RGBPub.inputs:type",          "rgb"),
            ("RGBPub.inputs:topicName",     "/camera/color/image_raw"),
            ("RGBPub.inputs:frameId",       "camera_color_optical_frame"),
            ("ColorInfoPub.inputs:topicName", "/camera/color/camera_info"),
            ("ColorInfoPub.inputs:frameId",   "camera_color_optical_frame"),
            ("DepthPub.inputs:type",        "depth"),
            ("DepthPub.inputs:topicName",   "/camera/depth/image_rect_raw"),
            ("DepthPub.inputs:frameId",     "camera_depth_optical_frame"),
            ("DepthInfoPub.inputs:topicName", "/camera/depth/camera_info"),
            ("DepthInfoPub.inputs:frameId",   "camera_depth_optical_frame"),
        ],
    },
)
og.Controller.evaluate_sync(ros_camera_graph)
simulation_app.update()
print("[INFO] D455 OmniGraph created -> /camera/color, /camera/depth")

# ── rclpy bridge node ─────────────────────────────────────────────────────────
class AsroBridge(Node):
    def __init__(self):
        super().__init__("asro_isaac_bridge")
        self.odom_pub  = self.create_publisher(Odometry, "/odom",  10)
        self.clock_pub = self.create_publisher(Clock,    "/clock", 10)
        self.imu_pub   = self.create_publisher(Imu,      "/imu",   10)
        self.tf_br     = TransformBroadcaster(self)
        self._v_lin    = 0.0
        self._omega    = 0.0
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)

        # The color/depth images are stamped with the optical frame ids set
        # above, but nothing publishes a TF for those frames on its own —
        # SLAM/rviz can't place the images relative to base_footprint/odom
        # without it ("could not transform frame", same issue the lidar hit).
        # The camera is rigidly fixed at a constant offset (real physics
        # FixedJoint), so one-shot static transforms are the correct fix.
        static_br = StaticTransformBroadcaster(self)
        static_transforms = []
        for child_frame in ("camera_color_optical_frame", "camera_depth_optical_frame"):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "base_footprint"
            t.child_frame_id = child_frame
            t.transform.translation.z = CAMERA_HEIGHT
            t.transform.rotation.x = _OPTICAL_ROTATION[0]
            t.transform.rotation.y = _OPTICAL_ROTATION[1]
            t.transform.rotation.z = _OPTICAL_ROTATION[2]
            t.transform.rotation.w = _OPTICAL_ROTATION[3]
            static_transforms.append(t)

        # IMU sits directly at base_footprint's origin (see IMU_PATH — it's
        # a child of ASRO_ROOT_PATH with zero local offset), so this static
        # transform is identity, not the optical-frame rotation above.
        imu_t = TransformStamped()
        imu_t.header.stamp = self.get_clock().now().to_msg()
        imu_t.header.frame_id = "base_footprint"
        imu_t.child_frame_id = "imu_link"
        imu_t.transform.rotation.w = 1.0
        static_transforms.append(imu_t)

        static_br.sendTransform(static_transforms)

    def _cmd_cb(self, msg: Twist):
        self._v_lin = msg.linear.x
        self._omega = msg.angular.z

    def step(self, pos, q_wxyz, v_lin, omega, sim_sec):
        sec  = int(sim_sec)
        nsec = int((sim_sec % 1) * 1e9)

        msg = Odometry()
        msg.header.stamp.sec     = sec
        msg.header.stamp.nanosec = nsec
        msg.header.frame_id      = "odom"
        msg.child_frame_id       = "base_footprint"
        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.position.z = float(pos[2])
        msg.pose.pose.orientation.x = float(q_wxyz[1])
        msg.pose.pose.orientation.y = float(q_wxyz[2])
        msg.pose.pose.orientation.z = float(q_wxyz[3])
        msg.pose.pose.orientation.w = float(q_wxyz[0])
        msg.twist.twist.linear.x  = v_lin
        msg.twist.twist.angular.z = omega
        self.odom_pub.publish(msg)

        clk = Clock()
        clk.clock.sec     = sec
        clk.clock.nanosec = nsec
        self.clock_pub.publish(clk)

        t = TransformStamped()
        t.header.stamp.sec     = sec
        t.header.stamp.nanosec = nsec
        t.header.frame_id = "odom"
        t.child_frame_id  = "base_footprint"
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        t.transform.rotation.x = float(q_wxyz[1])
        t.transform.rotation.y = float(q_wxyz[2])
        t.transform.rotation.z = float(q_wxyz[3])
        t.transform.rotation.w = float(q_wxyz[0])
        self.tf_br.sendTransform(t)

    def publish_imu(self, imu_frame, sim_sec):
        sec  = int(sim_sec)
        nsec = int((sim_sec % 1) * 1e9)

        msg = Imu()
        msg.header.stamp.sec     = sec
        msg.header.stamp.nanosec = nsec
        msg.header.frame_id      = "imu_link"
        # orientation stored [w, x, y, z]; ROS wants x, y, z, w
        o = imu_frame["orientation"]
        msg.orientation.w = float(o[0])
        msg.orientation.x = float(o[1])
        msg.orientation.y = float(o[2])
        msg.orientation.z = float(o[3])
        av = imu_frame["angular_velocity"]
        msg.angular_velocity.x = float(av[0])
        msg.angular_velocity.y = float(av[1])
        msg.angular_velocity.z = float(av[2])
        la = imu_frame["linear_acceleration"]
        msg.linear_acceleration.x = float(la[0])
        msg.linear_acceleration.y = float(la[1])
        msg.linear_acceleration.z = float(la[2])
        # Zero covariance = "unknown" per REP 145, consistent with everything
        # else in this bridge being unfiltered ground-truth-adjacent data.
        self.imu_pub.publish(msg)


# ── Simulate ──────────────────────────────────────────────────────────────────
SimulationManager.set_physics_dt(1.0 / 60.0)
app_utils.play()
simulation_app.update()

root = Articulation(ASRO_ROOT_PATH)
root.set_world_poses(
    positions=np.array([[0.0, 3.0, 0.3]]),
    orientations=np.array([[1.0, 0.0, 0.0, 0.0]])
)
root.set_velocities(
    linear_velocities=np.array([[0.0, 0.0, 0.0]]),
    angular_velocities=np.array([[0.0, 0.0, 0.0]])
)

for _ in range(30):
    SimulationManager.step()
    simulation_app.update()

rclpy.init()
bridge = AsroBridge()

frame = 0

print("[INFO] ASRo ROS2 bridge running")
print("[INFO]   Topics: /camera/color/image_raw /camera/color/camera_info "
      "/camera/depth/image_rect_raw /camera/depth/camera_info /imu /odom /tf /clock")
print("[INFO]   Drive:  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}'")

while simulation_app.is_running() and app_utils.is_playing():
    try:
        rclpy.spin_once(bridge, timeout_sec=0.0)
    except Exception:
        break

    pos, orient = root.get_world_poses()
    pos_np = pos.numpy()[0] if hasattr(pos, "numpy") else np.array(pos)[0]
    q = orient.numpy()[0] if hasattr(orient, "numpy") else np.array(orient)[0]
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    if not root.is_physics_tensor_entity_valid():
        app_utils.play()
        simulation_app.update()

    v_lin = bridge._v_lin
    omega = bridge._omega
    linear_deg  = float(np.clip(v_lin / 0.5, -1.0, 1.0)) * MAX_LINEAR_DEG
    angular_deg = float(np.clip(omega / 2.0, -1.0, 1.0)) * MAX_ANGULAR_DEG
    left_vel  = linear_deg - angular_deg
    right_vel = linear_deg + angular_deg
    for d in left_drives:
        d.GetTargetVelocityAttr().Set(left_vel)
    for d in right_drives:
        d.GetTargetVelocityAttr().Set(right_vel)

    # No manual transform write needed here — the camera is now a real rigid
    # body on a FixedJoint to the chassis (see setup above), so PhysX itself
    # carries its position and heading every step.

    SimulationManager.step()
    simulation_app.update()

    # Read Isaac's own authoritative simulation clock instead of hand-tracking
    # a separate sim_time += DT accumulator. The OmniGraph-driven camera
    # pipeline (ROS2CameraHelper, useSystemTime=False) stamps its messages
    # from this same internal clock, not from anything our Python loop
    # tracks — a manual accumulator drifted to roughly half this clock's
    # rate (root cause never fully isolated, likely simulation_app.update()
    # advancing time by more than one physics_dt per call), which showed up
    # downstream as RTAB-Map rejecting "extrapolation into the future" on
    # every odom->base_footprint TF lookup, since our /tf and /clock were
    # stamped on the slower, wrong clock while images used the real one.
    sim_time = SimulationManager.get_simulation_time()

    bridge.step(pos_np, q, v_lin, omega, sim_time)
    bridge.publish_imu(imu_sensor.get_data(), sim_time)
    frame += 1

    if frame % 300 == 0:
        print(f"[{sim_time:6.1f}s] x={pos_np[0]:.2f} y={pos_np[1]:.2f} "
              f"yaw={np.degrees(yaw):.1f}° cmd_v={v_lin:.2f} cmd_ω={omega:.2f}")

bridge.destroy_node()
rclpy.shutdown()
app_utils.stop()
simulation_app.close()
