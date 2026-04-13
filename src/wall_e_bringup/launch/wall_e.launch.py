from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='wall_e_bringup',
            executable='mega_bridge',
            name='mega_bridge',
            output='screen'
        ),

        Node(
            package='wall_e_bringup',
            executable='uno_bridge',
            name='uno_bridge',
            output='screen'
        ),

        Node(
            package='wall_e_bringup',
            executable='state_machine',
            name='state_machine',
            output='screen'
        ),

        Node(
            package='wall_e_bringup',
            executable='controller_node',
            name='controller_node',
            output='screen'
        ),
    ])
