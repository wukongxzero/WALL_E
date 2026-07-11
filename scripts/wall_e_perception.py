from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "anti_aliasing": 0})

import isaacsim.core.experimental.utils.app as app_utils

app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import omni
import omni.graph.core as og
from isaacsim.core.api import World
from pxr import Usd, UsdGeom, Gf

USD_PATH = "/home/wukong/WALL_E/usd/wall_e.usd/wall_e.usda"
ROBOT_PRIM = "/World/wall_e"
CHASSIS_PRIM = ROBOT_PRIM + "/Geometry/base_footprint/base_link"

world = World(stage_units_in_meters=1.0)

# load WALL-E USD into the scene
stage = omni.usd.get_context().get_stage()
ref_prim = stage.DefinePrim(ROBOT_PRIM, "Xform")
ref_prim.GetReferences().AddReference(USD_PATH)

world.reset()

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
            ("PubTFRobot",   "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ("SubCmdVel",    "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
        ],
        keys.SET_VALUES: [
            ("SimTime.inputs:resetOnStop",            True),
            ("Context.inputs:domain_id",              0),
            ("Clock.inputs:topicName",                "/clock"),
            ("ComputeOdom.inputs:chassisPrim",        CHASSIS_PRIM),
            ("PubOdom.inputs:topicName",              "/odom"),
            ("PubOdom.inputs:chassisFrameId",         "base_footprint"),
            ("PubTFOdom.inputs:childFrameId",         "odom"),
            ("PubTFOdom.inputs:parentFrameId",        "world"),
            ("PubTFRobot.inputs:targetPrims",         ROBOT_PRIM),
            ("PubTFRobot.inputs:topicName",           "/tf"),
            ("SubCmdVel.inputs:topicName",            "/cmd_vel"),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick",                   "Clock.inputs:execIn"),
            ("OnTick.outputs:tick",                   "ComputeOdom.inputs:execIn"),
            ("OnTick.outputs:tick",                   "PubTFOdom.inputs:execIn"),
            ("OnTick.outputs:tick",                   "PubTFRobot.inputs:execIn"),
            ("OnTick.outputs:tick",                   "SubCmdVel.inputs:execIn"),
            ("SimTime.outputs:simulationTime",        "Clock.inputs:timeStamp"),
            ("SimTime.outputs:simulationTime",        "PubOdom.inputs:timeStamp"),
            ("SimTime.outputs:simulationTime",        "PubTFOdom.inputs:timeStamp"),
            ("SimTime.outputs:simulationTime",        "PubTFRobot.inputs:timeStamp"),
            ("Context.outputs:context",               "Clock.inputs:context"),
            ("Context.outputs:context",               "PubOdom.inputs:context"),
            ("Context.outputs:context",               "PubTFOdom.inputs:context"),
            ("Context.outputs:context",               "PubTFRobot.inputs:context"),
            ("Context.outputs:context",               "SubCmdVel.inputs:context"),
            ("ComputeOdom.outputs:execOut",           "PubOdom.inputs:execIn"),
            ("ComputeOdom.outputs:position",          "PubOdom.inputs:position"),
            ("ComputeOdom.outputs:orientation",       "PubOdom.inputs:orientation"),
            ("ComputeOdom.outputs:linearVelocity",    "PubOdom.inputs:linearVelocity"),
            ("ComputeOdom.outputs:angularVelocity",   "PubOdom.inputs:angularVelocity"),
            ("ComputeOdom.outputs:position",          "PubTFOdom.inputs:translation"),
            ("ComputeOdom.outputs:orientation",       "PubTFOdom.inputs:rotation"),
        ],
    }
)

print("[OK] OmniGraph ROS2 bridge built")

# ── Run ────────────────────────────────────────────────────────────────────
print("[OK] Starting simulation loop — publishing /clock /odom /tf")
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
