import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('wall_e_bringup')
    rtabmap_params = os.path.join(pkg_path, 'config', 'rtabmap_asro.yaml')
    rviz_config = os.path.join(pkg_path, 'config', 'wall_e_nav.rviz')

    return LaunchDescription([

        # ── RTAB-MAP (RGBD mapping only, no Nav2) ──
        # Run ros2_asro.py separately first (standalone Isaac Sim script,
        # not ros2-launch-able) so the camera/odom/EKF topics already exist.
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_params, {'use_sim_time': True}],
            remappings=[
                ('rgb/image',         '/camera/color/image_raw'),
                ('rgb/camera_info',   '/camera/color/camera_info'),
                ('depth/image',       '/camera/depth/image_rect_raw'),
                ('depth/camera_info', '/camera/depth/camera_info'),
                # Consume the EKF's fused estimate, not raw ground-truth
                # odom — that's the whole reason ekf_asro.launch.py exists.
                ('odom',              '/odometry/filtered'),
            ],
            arguments=['--delete_db_on_start'],
        ),

        # ── RVIZ2 ──
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
