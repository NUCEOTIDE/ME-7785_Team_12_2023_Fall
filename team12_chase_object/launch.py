from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='team12_chase_object',
            executable='image_processor',
            name='find_object'
        ),
        Node(
            package='team12_chase_object',
            executable='lidar_processor',
            name='object_range'
        ),
        Node(
            package='team12_chase_object',
            executable='actuator',
            name='chase_object'
        ),
    ])
