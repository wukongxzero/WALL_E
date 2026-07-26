import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('wall_e_bringup')
    rtabmap_params = os.path.join(pkg_path, 'config', 'rtabmap.yaml')
    rviz_config = os.path.join(pkg_path, 'config', 'wall_e_nav.rviz')

    return LaunchDescription([

        # ── RTAB-MAP (RGBD mapping only, no Nav2) ──
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_params, {'use_sim_time': True}],
            remappings=[
                ('rgb/image',       '/camera/rgb/image_raw'),
                ('rgb/camera_info', '/camera/rgb/camera_info'),
                ('depth/image',     '/camera/depth/image_raw'),
                ('odom',            '/odom'),
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
