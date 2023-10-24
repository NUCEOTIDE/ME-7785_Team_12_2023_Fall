from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ntg',
            executable='actuator',
            name='chase_object'
        ),
        Node(
            package='ntg',
            executable='lidar_processor',
            name='get_object_range'
        ),

    ])
