from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='argos_final',
            executable='camara_node',
            name='camara_node',
            output='screen'
        ),
        Node(
            package='argos_final',
            executable='line_follower',
            name='line_follower',
            output='screen'
        ),
        Node(
            package='argos_final',
            executable='controllerf',
            name='controllerf',
            output='screen'
        ),
    ])
