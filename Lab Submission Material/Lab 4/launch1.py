from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ntg',
            executable='vector_generator',
            name='go_to_goal'
        ),

    ])
