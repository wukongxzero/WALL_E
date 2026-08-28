import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    pkg_path    = get_package_share_directory('wall_e_bringup')
    nav2_pkg    = get_package_share_directory('nav2_bringup')

    # Process URDF
    xacro_file        = os.path.join(pkg_path, 'urdf', 'wall_e.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # Config files
    nav2_params    = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    rtabmap_params = os.path.join(pkg_path, 'config', 'rtabmap.yaml')

    return LaunchDescription([

        # ── ROBOT DESCRIPTION ──
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),


        # ── SERIAL BRIDGES ──
        Node(
            package='wall_e_bringup',
            executable='mega_node',
            name='mega_node',
            output='screen'
        ),

        Node(
            package='wall_e_bringup',
            executable='uno_bridge',
            name='uno_bridge',
            output='screen'
        ),

        # ── STATE MACHINE ──
        Node(
            package='wall_e_bringup',
            executable='state_machine',
            name='state_machine',
            output='screen'
        ),
        # ── YOLO NAV ──
        Node(
            package='wall_e_bringup',
            executable='yolo_nav_node.py',
            name='yolo_nav_node',
            output='screen'
        ),

        # ── CONTROLLER ──
        Node(
            package='wall_e_bringup',
            executable='controller_node',
            name='controller_node',
            output='screen'
        ),

        # ── D435 REALSENSE ──
         Node(
             package='realsense2_camera',
             executable='realsense2_camera_node',
             name='realsense2_camera',
             parameters=[{
                 'depth_module.profile': '640x480x30',
                 'rgb_camera.profile':   '640x480x30',
                 'align_depth.enable':   True,
                 'pointcloud.enable':    True,
             }],
             output='screen'
         ),

        # ── DEPTH → LASERSCAN (for Nav2 costmaps) ──
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            parameters=[{
                'scan_height':   10,      # pixel rows to collapse into scan
                'range_min':     0.2,     # D435 min reliable depth
                'range_max':     4.0,
                'output_frame':  'camera_link',
            }],
            remappings=[
                ('depth',        '/camera/realsense2_camera/aligned_depth_to_color/image_raw'),
                ('depth_camera_info', '/camera/realsense2_camera/aligned_depth_to_color/camera_info'),
                ('scan',         '/scan'),
            ],
            output='screen'
        ),

        # ── RTAB-MAP ──
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_params],
            remappings=[
                ('rgb/image',       '/camera/realsense2_camera/color/image_raw'),
                ('rgb/camera_info', '/camera/realsense2_camera/color/camera_info'),
                ('depth/image',     '/camera/realsense2_camera/aligned_depth_to_color/image_raw'),
                ('odom',            '/odom'),
            ],
            arguments=['--delete_db_on_start']
        ),
	# ── WEBRTC VIDEO BRIDGE ──
	Node(
    	     package='wall_e_bringup',
    	     executable='webrtc_video_bridge.py',
    	     name='webrtc_video_bridge',
    	     output='screen'
	),
        # ── NAV2 ──
        # RTAB-Map provides /map and map→odom TF — no static map file needed
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'params_file':  nav2_params,
                'use_sim_time': 'false',
            }.items()
        ),

    ])
