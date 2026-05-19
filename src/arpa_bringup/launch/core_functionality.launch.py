from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction



def launch_setup(context, *args, **kwargs):

    use_sim_time = LaunchConfiguration("use_sim_time")
    core_functionality_node = Node(
        package="arpa_helper_tools",
        executable="core_functionality_node.py",
        name="core_functionality_node",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )
    return [core_functionality_node]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Enables simulation time if true.",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])