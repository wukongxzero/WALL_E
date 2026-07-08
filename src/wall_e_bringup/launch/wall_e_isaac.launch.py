import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    pkg_path = get_package_share_directory('wall_e_bringup')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    xacro_file        = os.path.join(pkg_path, 'urdf', 'wall_e.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    nav2_params    = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    rtabmap_params = os.path.join(pkg_path, 'config', 'rtabmap.yaml')
    rviz_config    = os.path.join(pkg_path, 'config', 'wall_e_nav.rviz')

    return LaunchDescription([

        # ── ROBOT DESCRIPTION ──
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
            output='screen'
        ),

        # ── STATE MACHINE ──
        Node(
            package='wall_e_bringup',
            executable='state_machine',
            name='state_machine',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # ── DEPTH IMAGE → LASER SCAN ──
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            parameters=[{
                'use_sim_time': True,
                'range_min':     0.3,
                'range_max':     4.0,
                'output_frame':  'camera_link',
            }],
            remappings=[
                ('depth',             '/camera/depth/image_raw'),
                ('depth_camera_info', '/camera/depth/camera_info'),
                ('scan',              '/scan'),
            ],
            output='screen'
        ),

        # ── RTAB-MAP ──
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_params, {'use_sim_time': True}],
            remappings=[
                ('rgb/image',         '/camera/rgb/image_raw'),
                ('rgb/camera_info',   '/camera/rgb/camera_info'),
                ('depth/image',       '/camera/depth/image_raw'),
                ('depth/camera_info', '/camera/depth/camera_info'),
                ('odom',              '/odom'),
            ],
            arguments=['--delete_db_on_start']
        ),

        # ── RVIZ2 ──
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # ── NAV2 ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_path, 'launch', 'nav2_navigation.launch.py')
            ),
            launch_arguments={
                'params_file':  nav2_params,
                'use_sim_time': 'true',
            }.items()
        ),

    ])
