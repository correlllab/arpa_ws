from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Force software rendering inside Docker (prevents nouveau GLX crash)
    os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

    pkg = FindPackageShare('arpa_description')
    urdf_path = PathJoinSubstitution([pkg, 'urdf', 'arpa_system.urdf.xacro'])
    rviz_config_path = PathJoinSubstitution([pkg, 'rviz', 'arpa_visual.rviz'])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}],
            output='screen'
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])
