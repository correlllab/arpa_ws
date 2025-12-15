from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
        package="arpa_gui",
        executable="arpa_gui",
        output="screen"
        )
])