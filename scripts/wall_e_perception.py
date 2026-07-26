from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "anti_aliasing": 0})

import isaacsim.core.experimental.utils.app as app_utils

app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import omni
import omni.graph.core as og
import usdrt.Sdf
from isaacsim.core.api import World
from pxr import Usd, UsdGeom, Gf

USD_PATH = "/home/wukong/WALL_E/usd/wall_e.usd/wall_e.usda"
ROBOT_PRIM = "/World/wall_e"
BASE_FOOTPRINT_PRIM = ROBOT_PRIM + "/Geometry/base_footprint"
CHASSIS_PRIM = BASE_FOOTPRINT_PRIM + "/base_link"
CAMERA_LINK_PRIM = CHASSIS_PRIM + "/camera_link"
CAMERA_OPTICAL_PRIM = CAMERA_LINK_PRIM + "/camera_color_optical_frame"
CAMERA_PRIM = CAMERA_OPTICAL_PRIM + "/Camera"

world = World(stage_units_in_meters=1.0)

# load WALL-E USD into the scene
stage = omni.usd.get_context().get_stage()
ref_prim = stage.DefinePrim(ROBOT_PRIM, "Xform")
ref_prim.GetReferences().AddReference(USD_PATH)

# D435 color camera: ~69.5 deg HFOV / ~55 deg VFOV at 640x480 (4:3).
# camera_color_optical_frame follows REP-103 (+Z forward, +Y down), but a USD
# Camera's local convention looks down -Z with +Y up, so a plain child would
# point backward. Rotating 180 deg about local X re-aims -Z at the parent's
# +Z (and flips the up vector to match), fixing both without touching the URDF.
camera = UsdGeom.Camera.Define(stage, CAMERA_PRIM)
UsdGeom.XformCommonAPI(camera).SetRotate((180, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 20.0))
camera.CreateFocalLengthAttr(15.1)
camera.CreateHorizontalApertureAttr(20.955)
camera.CreateVerticalApertureAttr(15.7)

world.reset()

# ── OmniGraph: RGBD camera publishing (rgb + depth + camera_info) ─────────
ros_camera_graph, _, _, _ = og.Controller.edit(
    {
        "graph_path": "/Graph/ROS_Camera",
        "evaluator_name": "push",
        "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
    },
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnTick",           "omni.graph.action.OnTick"),
            ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("PubRGB",           "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("PubDepth",         "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("PubCamInfo",       "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnTick.outputs:tick",                      "CreateRenderProduct.inputs:execIn"),
            ("CreateRenderProduct.outputs:execOut",      "PubRGB.inputs:execIn"),
            ("CreateRenderProduct.outputs:execOut",      "PubDepth.inputs:execIn"),
            ("CreateRenderProduct.outputs:execOut",      "PubCamInfo.inputs:execIn"),
            ("CreateRenderProduct.outputs:renderProductPath", "PubRGB.inputs:renderProductPath"),
            ("CreateRenderProduct.outputs:renderProductPath", "PubDepth.inputs:renderProductPath"),
            ("CreateRenderProduct.outputs:renderProductPath", "PubCamInfo.inputs:renderProductPath"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("CreateRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(CAMERA_PRIM)]),
            ("CreateRenderProduct.inputs:width",  640),
            ("CreateRenderProduct.inputs:height", 480),
            ("PubRGB.inputs:type",       "rgb"),
            ("PubRGB.inputs:topicName",  "camera/rgb/image_raw"),
            ("PubRGB.inputs:frameId",    "camera_color_optical_frame"),
            ("PubDepth.inputs:type",      "depth"),
            ("PubDepth.inputs:topicName", "camera/depth/image_raw"),
            ("PubDepth.inputs:frameId",   "camera_color_optical_frame"),
            ("PubCamInfo.inputs:topicName", "camera/rgb/camera_info"),
            ("PubCamInfo.inputs:frameId",   "camera_color_optical_frame"),
        ],
    },
)
og.Controller.evaluate_sync(ros_camera_graph)

print("[OK] ROS2 camera graph built — publishing camera/rgb and camera/depth")

# ── OmniGraph: clock + odom + ROS2 bridge ──────────────────────────────────
keys = og.Controller.Keys

og.Controller.edit(
    {"graph_path": "/Graph/ROS_WALL_E", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnTick",       "omni.graph.action.OnPlaybackTick"),
            ("SimTime",      "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Context",      "isaacsim.ros2.bridge.ROS2Context"),
            ("Clock",        "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("ComputeOdom",  "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("PubOdom",      "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("PubTFOdom",    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ("PubTFFootprint", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ("PubTFRobot",   "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ("SubCmdVel",    "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("BreakLinVel",  "omni.graph.nodes.BreakVector3"),
            ("BreakAngVel",  "omni.graph.nodes.BreakVector3"),
            ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
            ("ArtController", "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        keys.SET_VALUES: [
            ("SimTime.inputs:resetOnStop",            True),
            ("Context.inputs:domain_id",              0),
            ("Clock.inputs:topicName",                "/clock"),
            ("ComputeOdom.inputs:chassisPrim",        CHASSIS_PRIM),
            ("PubOdom.inputs:topicName",              "/odom"),
            ("PubOdom.inputs:chassisFrameId",         "base_footprint"),
            ("PubTFOdom.inputs:childFrameId",         "base_footprint"),
            ("PubTFOdom.inputs:parentFrameId",        "odom"),
            ("PubTFFootprint.inputs:childFrameId",    "base_link"),
            ("PubTFFootprint.inputs:parentFrameId",   "base_footprint"),
            # CHASSIS_PRIM's articulation tree covers the tracks; camera/imu links
            # are separate fixed rigid bodies, not part of that articulation, so
            # ROS2PublishTransformTree needs them listed explicitly or their
            # frames never appear on /tf (rtabmap needs camera_color_optical_frame).
            ("PubTFRobot.inputs:targetPrims",         [
                CHASSIS_PRIM,
                CAMERA_LINK_PRIM,
                CAMERA_OPTICAL_PRIM,
                CAMERA_LINK_PRIM + "/camera_depth_optical_frame",
                CHASSIS_PRIM + "/imu_link",
            ]),
            # parentPrim must be a valid physics rigid body — base_footprint is a
            # plain bookkeeping Xform with no RigidBodyAPI (PoseTree errors with
            # "getObjectType eInvalid" if given it), so this has to stay base_link.
            # That means this subtree's root frame is literally named "base_link",
            # disconnected from PubTFOdom's "base_footprint" — bridged below via
            # PubTFFootprint (base_footprint and base_link are already coincident
            # in this graph: ComputeOdom reports CHASSIS_PRIM's pose as "base_footprint").
            ("PubTFRobot.inputs:parentPrim",          CHASSIS_PRIM),
            ("PubTFRobot.inputs:topicName",           "/tf"),
            ("SubCmdVel.inputs:topicName",            "/cmd_vel"),
            ("DiffController.inputs:wheelRadius",     0.08),
            ("DiffController.inputs:wheelDistance",   0.46),
            ("ArtController.inputs:targetPrim",       CHASSIS_PRIM),
            ("ArtController.inputs:jointNames",       ["left_track_joint", "right_track_joint"]),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick",                   "Clock.inputs:execIn"),
            ("OnTick.outputs:tick",                   "ComputeOdom.inputs:execIn"),
            ("OnTick.outputs:tick",                   "PubTFOdom.inputs:execIn"),
            ("OnTick.outputs:tick",                   "PubTFFootprint.inputs:execIn"),
            ("OnTick.outputs:tick",                   "PubTFRobot.inputs:execIn"),
            ("OnTick.outputs:tick",                   "SubCmdVel.inputs:execIn"),
            ("SimTime.outputs:simulationTime",        "Clock.inputs:timeStamp"),
            ("SimTime.outputs:simulationTime",        "PubOdom.inputs:timeStamp"),
            ("SimTime.outputs:simulationTime",        "PubTFOdom.inputs:timeStamp"),
            ("SimTime.outputs:simulationTime",        "PubTFFootprint.inputs:timeStamp"),
            ("SimTime.outputs:simulationTime",        "PubTFRobot.inputs:timeStamp"),
            ("Context.outputs:context",               "Clock.inputs:context"),
            ("Context.outputs:context",               "PubOdom.inputs:context"),
            ("Context.outputs:context",               "PubTFOdom.inputs:context"),
            ("Context.outputs:context",               "PubTFFootprint.inputs:context"),
            ("Context.outputs:context",               "PubTFRobot.inputs:context"),
            ("Context.outputs:context",               "SubCmdVel.inputs:context"),
            ("ComputeOdom.outputs:execOut",           "PubOdom.inputs:execIn"),
            ("ComputeOdom.outputs:position",          "PubOdom.inputs:position"),
            ("ComputeOdom.outputs:orientation",       "PubOdom.inputs:orientation"),
            ("ComputeOdom.outputs:linearVelocity",    "PubOdom.inputs:linearVelocity"),
            ("ComputeOdom.outputs:angularVelocity",   "PubOdom.inputs:angularVelocity"),
            ("ComputeOdom.outputs:position",          "PubTFOdom.inputs:translation"),
            ("ComputeOdom.outputs:orientation",       "PubTFOdom.inputs:rotation"),
            ("OnTick.outputs:tick",                   "ArtController.inputs:execIn"),
            ("SubCmdVel.outputs:execOut",             "DiffController.inputs:execIn"),
            ("SubCmdVel.outputs:linearVelocity",      "BreakLinVel.inputs:tuple"),
            ("BreakLinVel.outputs:x",                 "DiffController.inputs:linearVelocity"),
            ("SubCmdVel.outputs:angularVelocity",     "BreakAngVel.inputs:tuple"),
            ("BreakAngVel.outputs:z",                 "DiffController.inputs:angularVelocity"),
            ("DiffController.outputs:velocityCommand", "ArtController.inputs:velocityCommand"),
        ],
    }
)

print("[OK] OmniGraph ROS2 bridge built")

# ── Run ────────────────────────────────────────────────────────────────────
print("[OK] Starting simulation loop — publishing /clock /odom /tf")
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
