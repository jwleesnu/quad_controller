from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ui_controller',
            executable='ui_controller.py',
            output='screen'
        )
    ])