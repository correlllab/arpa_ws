from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_bag_arg = DeclareLaunchArgument(
        'input_bag',
        description='Path to the input ROS2 bag to play'
    )
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='.',
        description='Directory to save chunked bags'
    )

    bag_chunker_node = Node(
        package='arpa_helper_tools',
        executable='bag_chunker.py',
        name='bag_chunker',
        parameters=[{
            'input_bag': LaunchConfiguration('input_bag'),
            'output_dir': LaunchConfiguration('output_dir'),
        }],
        output='screen',
    )

    play_bag = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', LaunchConfiguration('input_bag')],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        input_bag_arg,
        output_dir_arg,
        bag_chunker_node,
        play_bag,
    ])
