from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='puzzlebot_closed_loop',
            executable='puzzlebot_odometry',
            name='puzzlebot_odometry',
            output='screen'
        ),
        Node(
            package='puzzlebot_closed_loop',
            executable='controller',
            name='controller',
            output='screen'
        ),
        Node(
            package='puzzlebot_closed_loop',
            executable='path_generator',
            name='path_generator',
            output='screen'
        ),
        Node(
            package='puzzlebot_closed_loop',
            executable='prueba',
            name='prueba',
            output='screen'
        )
    ])