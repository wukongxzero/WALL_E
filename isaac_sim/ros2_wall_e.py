"""
WALL-E Isaac Sim 6.0 — ROS2 bridge for SLAM + autonomous navigation.

Publishes:
  /camera/rgb/image_raw          (sensor_msgs/Image, 30 Hz)
  /camera/depth/image_raw        (sensor_msgs/Image, 30 Hz)
  /camera/rgb/camera_info        (sensor_msgs/CameraInfo, 30 Hz)
  /camera/depth/camera_info      (sensor_msgs/CameraInfo, 30 Hz)
  /odom                          (nav_msgs/Odometry, 60 Hz)
  /tf                            (odom → base_footprint, dynamic)
  /tf_static                     (base_footprint→base_link, base_link→camera_link, optical frames)
  /clock                         (rosgraph_msgs/Clock, 60 Hz)

Subscribes:
  /cmd_vel                       (geometry_msgs/Twist)

Run with:
  source /opt/ros/jazzy/setup.bash
  ~/isaac-sim/python.sh ~/WALL_E/isaac_sim/ros2_wall_e.py
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import math
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.objects import DistantLight, GroundPlane
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.experimental.prims import RigidPrim
import omni.usd
import omni.graph.core as og
import numpy as np
from pxr import UsdGeom, UsdPhysics, Gf
import usdrt.Sdf

# ── Enable ROS2 bridge (must happen before any rclpy import) ──────────────────
app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

WALL_E_USD       = "/home/wukong/WALL_E/usd/wall_e_v2.usd"
WHEEL_RADIUS     = 0.08    # m
K_YAW            = 5.0     # P-gain — holds heading when cmd_vel omega ≈ 0
CAMERA_PATH      = "/World/WallECamera"   # top-level — IsaacCreateRenderProduct requires this
ROS_CAMERA_GRAPH = "/World/ROS2_Camera"
# camera_link offset from base_footprint (from URDF static TFs)
CAM_FORWARD = 0.15   # m forward of base_link center
CAM_HEIGHT  = 0.3675 # m above ground (base_footprint z=0): 0.2025 + 0.165

# ── Scene ─────────────────────────────────────────────────────────────────────
stage_utils.create_new_stage()
stage_utils.set_stage_units(meters_per_unit=1.0)
GroundPlane("/World/GroundPlane")
DistantLight("/World/DistantLight").set_intensities(3000)

stage = omni.usd.get_context().get_stage()
wall_e_prim = stage.DefinePrim("/World/wall_e", "Xform")
wall_e_prim.GetReferences().AddReference(WALL_E_USD)

# ── Room: 10×10m enclosed space + colored landmark boxes ─────────────────────
def make_box(path, pos, half_extents, color):
    p = stage.DefinePrim(path, "Cube")
    UsdGeom.Cube(p).GetSizeAttr().Set(2.0)
    UsdGeom.Cube(p).GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
    xf = UsdGeom.Xformable(p)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddScaleOp().Set(Gf.Vec3f(*half_extents))
    UsdPhysics.CollisionAPI.Apply(p)

make_box("/World/Env/WallN", (0,   5.15, 1.0), (5.15, 0.15, 1.0), (0.6, 0.6, 0.6))
make_box("/World/Env/WallS", (0,  -5.15, 1.0), (5.15, 0.15, 1.0), (0.6, 0.6, 0.6))
make_box("/World/Env/WallE", (5.15,  0,  1.0), (0.15, 5.0,  1.0), (0.6, 0.6, 0.6))
make_box("/World/Env/WallW", (-5.15, 0,  1.0), (0.15, 5.0,  1.0), (0.6, 0.6, 0.6))
make_box("/World/Env/Red",    (3,   2,  0.25), (0.25, 0.25, 0.25), (0.9, 0.1, 0.1))
make_box("/World/Env/Green",  (-3,  2,  0.25), (0.25, 0.25, 0.25), (0.1, 0.8, 0.1))
make_box("/World/Env/Blue",   (0,  -3,  0.25), (0.25, 0.25, 0.25), (0.1, 0.2, 0.9))
make_box("/World/Env/Yellow", (3,  -3,  0.25), (0.25, 0.25, 0.25), (0.9, 0.8, 0.1))
make_box("/World/Env/Purple", (-3, -2,  0.25), (0.25, 0.25, 0.25), (0.7, 0.1, 0.7))

simulation_app.update()

# ── Track joints ──────────────────────────────────────────────────────────────
for prim in stage.TraverseAll():
    if prim.GetName() in ("left_track_joint", "right_track_joint"):
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTypeAttr("velocity")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(1.0)
        drive.CreateMaxForceAttr(20.0)
        drive.CreateTargetVelocityAttr(0.0)

# ── Camera prim at top level (required by IsaacCreateRenderProduct) ───────────
from pxr import Sdf
cam_prim = UsdGeom.Camera(stage.DefinePrim(CAMERA_PATH, "Camera"))
cam_xform = UsdGeom.XformCommonAPI(cam_prim)
cam_xform.SetTranslate(Gf.Vec3d(CAM_FORWARD, 0.0, CAM_HEIGHT))
cam_prim.GetHorizontalApertureAttr().Set(20.955)
cam_prim.GetVerticalApertureAttr().Set(15.2908)
cam_prim.GetProjectionAttr().Set("perspective")
cam_prim.GetFocalLengthAttr().Set(24.0)
cam_prim.GetFocusDistanceAttr().Set(4.0)
cam_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 10.0))
cam_prim.GetPrim().CreateAttribute("exposure:time",         Sdf.ValueTypeNames.Float).Set(0.02)
cam_prim.GetPrim().CreateAttribute("exposure:responsivity", Sdf.ValueTypeNames.Float).Set(1.10267)
cam_prim.GetPrim().CreateAttribute("exposure:fStop",        Sdf.ValueTypeNames.Float).Set(5.0)
simulation_app.update()

# ── OmniGraph: camera rgb + depth + info (mirrors official NVIDIA example) ────
keys = og.Controller.Keys
ros_camera_graph, _, _, _ = og.Controller.edit(
    {
        "graph_path": ROS_CAMERA_GRAPH,
        "evaluator_name": "push",
        "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
    },
    {
        keys.CREATE_NODES: [
            ("OnTick",        "omni.graph.action.OnTick"),
            ("CreateRP",      "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("CamRgb",        "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("CamDepth",      "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("CamInfo",       "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ("CamDepthInfo",  "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick",                "CreateRP.inputs:execIn"),
            ("CreateRP.outputs:execOut",           "CamRgb.inputs:execIn"),
            ("CreateRP.outputs:execOut",           "CamDepth.inputs:execIn"),
            ("CreateRP.outputs:execOut",           "CamInfo.inputs:execIn"),
            ("CreateRP.outputs:execOut",           "CamDepthInfo.inputs:execIn"),
            ("CreateRP.outputs:renderProductPath", "CamRgb.inputs:renderProductPath"),
            ("CreateRP.outputs:renderProductPath", "CamDepth.inputs:renderProductPath"),
            ("CreateRP.outputs:renderProductPath", "CamInfo.inputs:renderProductPath"),
            ("CreateRP.outputs:renderProductPath", "CamDepthInfo.inputs:renderProductPath"),
        ],
        keys.SET_VALUES: [
            ("CreateRP.inputs:cameraPrim", [usdrt.Sdf.Path(CAMERA_PATH)]),
            ("CreateRP.inputs:width",  1280),  # 1280x720 keeps DLSS above minimum input resolution
            ("CreateRP.inputs:height",  720),
            ("CamRgb.inputs:type",              "rgb"),
            ("CamRgb.inputs:topicName",         "/camera/rgb/image_raw"),
            ("CamRgb.inputs:frameId",           "camera_link"),
            ("CamDepth.inputs:type",            "depth"),
            ("CamDepth.inputs:topicName",       "/camera/depth/image_raw"),
            ("CamDepth.inputs:frameId",         "camera_depth_optical_frame"),
            ("CamInfo.inputs:topicName",        "/camera/rgb/camera_info"),
            ("CamInfo.inputs:frameId",          "camera_link"),
            ("CamDepthInfo.inputs:topicName",   "/camera/depth/camera_info"),
            ("CamDepthInfo.inputs:frameId",     "camera_depth_optical_frame"),
        ],
    },
)
og.Controller.evaluate_sync(ros_camera_graph)
simulation_app.update()

# ── Gate paths to control camera publish rate ─────────────────────────────────
render_product_path = og.Controller.attribute(
    f"{ROS_CAMERA_GRAPH}/CreateRP.outputs:renderProductPath"
).get()
print(f"[DEBUG] render_product_path = '{render_product_path}'")

import omni.syntheticdata._syntheticdata as sd
rv_rgb   = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
rv_depth = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.DistanceToImagePlane.name)
rgb_gate_path   = omni.syntheticdata.SyntheticData._get_node_path(rv_rgb   + "IsaacSimulationGate", render_product_path)
depth_gate_path = omni.syntheticdata.SyntheticData._get_node_path(rv_depth + "IsaacSimulationGate", render_product_path)
info_gate_path  = omni.syntheticdata.SyntheticData._get_node_path("PostProcessDispatch" + "IsaacSimulationGate", render_product_path)
print(f"[DEBUG] rgb_gate_path = '{rgb_gate_path}'")

# Publish RGB every 2 frames (~30Hz), depth every 2 frames, info every frame
og.Controller.attribute(rgb_gate_path   + ".inputs:step").set(2)
og.Controller.attribute(depth_gate_path + ".inputs:step").set(2)
og.Controller.attribute(info_gate_path  + ".inputs:step").set(1)


# ── rclpy bridge node ─────────────────────────────────────────────────────────
class WallEBridge(Node):
    def __init__(self):
        super().__init__("wall_e_isaac_bridge")
        self.odom_pub  = self.create_publisher(Odometry, "/odom",  10)
        self.clock_pub = self.create_publisher(Clock,    "/clock", 10)
        self.tf_br     = TransformBroadcaster(self)
        self.stf_br    = StaticTransformBroadcaster(self)
        self._v_lin    = 0.0
        self._omega    = 0.0
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self._publish_static_tfs()

    def _cmd_cb(self, msg: Twist):
        self._v_lin = msg.linear.x
        self._omega = msg.angular.z

    def _publish_static_tfs(self):
        now = self.get_clock().now().to_msg()

        def make_tf(parent, child, xyz, rpy):
            t = TransformStamped()
            t.header.stamp    = now
            t.header.frame_id = parent
            t.child_frame_id  = child
            t.transform.translation.x = float(xyz[0])
            t.transform.translation.y = float(xyz[1])
            t.transform.translation.z = float(xyz[2])
            r, p, y = float(rpy[0]), float(rpy[1]), float(rpy[2])
            cr, sr = math.cos(r/2), math.sin(r/2)
            cp, sp = math.cos(p/2), math.sin(p/2)
            cy, sy = math.cos(y/2), math.sin(y/2)
            t.transform.rotation.x = sr*cp*cy - cr*sp*sy
            t.transform.rotation.y = cr*sp*cy + sr*cp*sy
            t.transform.rotation.z = cr*cp*sy - sr*sp*cy
            t.transform.rotation.w = cr*cp*cy + sr*sp*sy
            return t

        self.stf_br.sendTransform([
            make_tf("base_footprint", "base_link", (0, 0, 0.2025), (0, 0, 0)),
            make_tf("base_link", "camera_link", (0.15, 0, 0.165), (0, 0, 0)),
            make_tf("camera_link", "camera_depth_optical_frame", (0, 0, 0), (-math.pi/2, 0, -math.pi/2)),
            make_tf("camera_link", "camera_color_optical_frame", (0, 0, 0), (-math.pi/2, 0, -math.pi/2)),
        ])

    def step(self, pos, q_wxyz, v_lin, omega, sim_sec):
        sec  = int(sim_sec)
        nsec = int((sim_sec % 1) * 1e9)

        msg = Odometry()
        msg.header.stamp.sec      = sec
        msg.header.stamp.nanosec  = nsec
        msg.header.frame_id       = "odom"
        msg.child_frame_id        = "base_footprint"
        msg.pose.pose.position.x  = float(pos[0])
        msg.pose.pose.position.y  = float(pos[1])
        msg.pose.pose.position.z  = float(pos[2])
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
        t.header.stamp.sec      = sec
        t.header.stamp.nanosec  = nsec
        t.header.frame_id  = "odom"
        t.child_frame_id   = "base_footprint"
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        t.transform.rotation.x = float(q_wxyz[1])
        t.transform.rotation.y = float(q_wxyz[2])
        t.transform.rotation.z = float(q_wxyz[3])
        t.transform.rotation.w = float(q_wxyz[0])
        self.tf_br.sendTransform(t)


# ── Simulation setup + main loop ──────────────────────────────────────────────
SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
app_utils.play()
simulation_app.update()

rclpy.init()
bridge = WallEBridge()
root = RigidPrim("/World/wall_e/base_footprint")

sim_time   = 0.0
DT         = 1.0 / 60.0
target_yaw = 0.0
frame      = 0

print("[INFO] WALL-E ROS2 bridge running")
print("[INFO]   Topics: /odom /tf /tf_static /clock /camera/rgb/image_raw /camera/depth/image_raw")
print("[INFO]   Drive:  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}'")

while simulation_app.is_running() and app_utils.is_playing():
    try:
        rclpy.spin_once(bridge, timeout_sec=0.0)
    except Exception:
        break

    pos, orient = root.get_world_poses()
    pos_np = pos.numpy()[0] if hasattr(pos, "numpy") else np.array(pos)[0]
    q      = orient.numpy()[0] if hasattr(orient, "numpy") else np.array(orient)[0]

    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    v_lin = bridge._v_lin
    omega = bridge._omega

    if abs(omega) > 0.05:
        target_yaw = yaw
    yaw_err  = (target_yaw - yaw + np.pi) % (2 * np.pi) - np.pi
    omega_cmd = omega + K_YAW * yaw_err

    vx = v_lin * np.cos(yaw)
    vy = v_lin * np.sin(yaw)
    root.set_velocities(
        linear_velocities=np.array([[vx, vy, 0.0]]),
        angular_velocities=np.array([[0.0, 0.0, omega_cmd]])
    )

    bridge.step(pos_np, q, v_lin, omega, sim_time)

    # ── Keep camera prim tracking the robot ──────────────────────────────
    cam_x = pos_np[0] + CAM_FORWARD * np.cos(yaw)
    cam_y = pos_np[1] + CAM_FORWARD * np.sin(yaw)
    cam_xform.SetTranslate(Gf.Vec3d(float(cam_x), float(cam_y), CAM_HEIGHT))
    cam_xform.SetRotate((0.0, 0.0, float(np.degrees(yaw))),
                        UsdGeom.XformCommonAPI.RotationOrderXYZ)

    simulation_app.update()
    sim_time += DT
    frame    += 1

    if frame % 300 == 0:
        print(f"[{sim_time:6.1f}s] x={pos_np[0]:.2f} y={pos_np[1]:.2f} "
              f"yaw={np.degrees(yaw):.1f}° cmd_v={v_lin:.2f} cmd_ω={omega:.2f}")

bridge.destroy_node()
rclpy.shutdown()
app_utils.stop()
simulation_app.close()
