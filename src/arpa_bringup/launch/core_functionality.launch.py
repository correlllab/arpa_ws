from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import OpaqueFunction



def launch_setup(context, *args, **kwargs):

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_collision_plane = LaunchConfiguration("use_collision_plane")
    core_functionality_node = Node(
        package="arpa_helper_tools",
        executable="core_functionality_node.py",
        name="core_functionality_node",
        parameters=[{
            "use_sim_time": use_sim_time,
            "use_collision_plane": use_collision_plane,
        }],
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_collision_plane",
            # Default tracks use_sim_time: sim -> no plane, real -> plane.
            # Only flip use_sim_time; this follows automatically. Override
            # explicitly only for unusual setups.
            default_value=PythonExpression(
                ["'false' if '", LaunchConfiguration("use_sim_time"), "' == 'true' else 'true'"]
            ),
            description="Add the table collision plane to the planning scene. "
                        "Defaults to the opposite of use_sim_time (off in sim, on for real).",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])