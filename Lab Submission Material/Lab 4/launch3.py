from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ntg',
            executable='parameter_changer',
            name='change_parameter'
        ),

    ])
