from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='brain_module',
            executable='brain_node',
            name='brain_node',
            output='screen',
        )
    ])