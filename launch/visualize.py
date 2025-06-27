from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_uroc',
            executable='foxglove_3d_path_visualization',
            name='foxglove_3d_path_visualization'
        )
    ])
