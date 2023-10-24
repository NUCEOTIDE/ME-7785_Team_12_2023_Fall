from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ntg',
            executable='odom_reseter',
            name='print_fixed_odom'
        ),

    ])
