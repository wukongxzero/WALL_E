import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    pkg_path = get_package_share_directory('wall_e_bringup')

    xacro_file        = os.path.join(pkg_path, 'urdf', 'wall_e.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    nav2_params    = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    rtabmap_params = os.path.join(pkg_path, 'config', 'rtabmap.yaml')
    rviz_config    = os.path.join(pkg_path, 'config', 'wall_e_nav.rviz')

    bt_xml = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml')

    nav2_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

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
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[nav2_params, {'use_sim_time': True}],
            remappings=nav2_remappings + [('cmd_vel', '/cmd_vel')],
            output='screen'
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            parameters=[nav2_params, {'use_sim_time': True}],
            remappings=nav2_remappings,
            output='screen'
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[nav2_params, {'use_sim_time': True}],
            remappings=nav2_remappings,
            output='screen'
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[nav2_params, {'use_sim_time': True}],
            remappings=nav2_remappings,
            output='screen'
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[nav2_params, {
                'use_sim_time': True,
                'default_nav_to_pose_bt_xml': bt_xml,
            }],
            remappings=nav2_remappings,
            output='screen'
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            parameters=[nav2_params, {'use_sim_time': True}],
            remappings=nav2_remappings + [
                ('cmd_vel', 'nav_cmd_vel'),
                ('cmd_vel_smoothed', 'nav_cmd_vel'),
            ],
            output='screen'
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'velocity_smoother',
                ],
            }],
            output='screen'
        ),

    ])
