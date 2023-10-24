from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ntg',
            executable='lidar_viewer',
            name='turtlebot3_range'
        ),
        Node(
            package='ntg',
            executable='vector_viewer',
            name='turtlebot3_vectors'
        ),
    ])
