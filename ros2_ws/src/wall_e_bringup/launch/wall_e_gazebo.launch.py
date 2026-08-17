import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    pkg_path       = get_package_share_directory('wall_e_bringup')
    gz_ros_pkg     = get_package_share_directory('ros_gz_sim')

    xacro_file        = os.path.join(pkg_path, 'urdf', 'wall_e.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    world_file        = os.path.join(pkg_path, 'worlds', 'test_room.sdf')
    rtabmap_params    = os.path.join(pkg_path, 'config', 'rtabmap.yaml')
    nav2_params       = os.path.join(pkg_path, 'config', 'nav2_params.yaml')

    bt_xml = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml')

    nav2_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    return LaunchDescription([

        # ── GAZEBO ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_ros_pkg, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'{world_file} -r'}.items()
        ),

        # ── ROBOT STATE PUBLISHER ──
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True}],
            output='screen'
        ),

        # ── SPAWN WALL-E ──
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'wall_e',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        ),

        # ── ROS-GZ BRIDGE ──
        # Bridges: /cmd_vel, /odom, /clock, /tf, camera topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/world/test_room/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                '/wall_e/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/wall_e/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/wall_e/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/world/test_room/model/wall_e/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            remappings=[
                ('/world/test_room/model/wall_e/joint_state', '/joint_states'),
                ('/world/test_room/clock', '/clock'),
            ],
            output='screen'
        ),

        # ── DEPTH → LASERSCAN ──
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            parameters=[{
                'scan_height':  10,
                'range_min':    0.3,
                'range_max':    4.0,
                'output_frame': 'camera_link',
                'use_sim_time': True,
            }],
            remappings=[
                ('depth',             '/wall_e/camera/depth_image'),
                ('depth_camera_info', '/wall_e/camera/camera_info'),
                ('scan',              '/scan'),
            ],
            output='screen'
        ),

        # ── ODOM → TF BROADCASTER ──
        Node(
            package='wall_e_bringup',
            executable='odom_to_tf.py',
            name='odom_to_tf',
            parameters=[{'use_sim_time': True}],
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
                ('rgb/image',       '/wall_e/camera/image'),
                ('rgb/camera_info', '/wall_e/camera/camera_info'),
                ('depth/image',     '/wall_e/camera/depth_image'),
                ('odom',            '/odom'),
                ('scan',            '/scan'),
            ],
            arguments=['--delete_db_on_start']
        ),

        # ── STATE MACHINE ──
        Node(
            package='wall_e_bringup',
            executable='state_machine',
            name='state_machine',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('/camera/realsense2_camera/color/image_raw', '/wall_e/camera/image'),
            ],
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
