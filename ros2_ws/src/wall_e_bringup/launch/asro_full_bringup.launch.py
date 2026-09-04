"""Sequenced ASRo bringup — replaces the manual three-terminal launch order
(ros2_asro.py -> ekf_asro.launch.py -> slam_test_asro.launch.py) with
event-based gating, so each stage only starts once the previous stage's
topics actually exist instead of relying on the human to wait and type
the next command in the right order.

Requires a readiness-check node (not yet written — see TODO below) that
takes a `topic_name` parameter, subscribes to it, and exits with code 0
on the first message received. Its process *exiting* is what gates the
next stage via OnProcessExit — a long-running node wouldn't work here,
the whole point is that it terminates once its condition is met.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

ISAAC_SIM_PYTHON = os.path.expanduser('~/WALL_E/isaac-sim/python.sh')
ROS2_ASRO_SCRIPT = os.path.expanduser('~/WALL_E/simulation/isaac_sim/ros2_asro.py')


def readiness_check(topic_name: str, name: str) -> Node:
    # TODO (user): write this node. Package: wall_e_bringup.
    # Executable name below assumes it'll be installed as readiness_check.py
    # via CMakeLists.txt's install(PROGRAMS ...), same pattern as the other
    # Python nodes in src/. Behavior: subscribe to `topic_name` param, exit
    # 0 on first message received -- must actually terminate, not spin forever.
    return Node(
        package='wall_e_bringup',
        executable='readiness_check.py',
        name=name,
        output='screen',
        parameters=[{'topic_name': topic_name}],
    )


def generate_launch_description():
    pkg_path = get_package_share_directory('wall_e_bringup')

    # ── STAGE 1: Isaac Sim standalone script ──
    # Not a ROS2-launchable target (no rclpy.init() gating its own startup
    # the way a Node action expects) -- ExecuteProcess runs it as a raw
    # subprocess instead.
    isaac_sim = ExecuteProcess(
        cmd=[ISAAC_SIM_PYTHON, ROS2_ASRO_SCRIPT],
        output='screen',
    )

    # Gate: wait for /odom to actually be published before starting EKF --
    # this is the condition ekf_asro.launch.py's own comment already
    # documents as required, just previously enforced by a human waiting.
    wait_for_odom = readiness_check('/odom', 'wait_for_odom')

    ekf_stage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'ekf_asro.launch.py')
        ),
    )

    # Gate: wait for /odometry/filtered (EKF's output) before starting
    # RTAB-Map -- slam_test_asro.launch.py's rtabmap node remaps its odom
    # input to this exact topic, so it must exist first.
    wait_for_ekf = readiness_check('/odometry/filtered', 'wait_for_ekf_output')

    slam_stage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'slam_test_asro.launch.py')
        ),
    )

    return LaunchDescription([
        isaac_sim,
        wait_for_odom,

        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_for_odom,
                on_exit=[ekf_stage, wait_for_ekf],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_for_ekf,
                on_exit=[slam_stage],
            )
        ),
    ])
