import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('wall_e_bringup')
    ekf_params = os.path.join(pkg_path, 'config', 'ekf_asro.yaml')

    return LaunchDescription([

        # ── EKF (robot_localization) — fuses /odom + /imu -> /odometry/filtered ──
        # Run ros2_asro.py separately first (it's a standalone Isaac Sim
        # script, not a ROS2 launch target) so /odom and /imu already exist
        # before this node starts subscribing to them.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params],
        ),
    ])
