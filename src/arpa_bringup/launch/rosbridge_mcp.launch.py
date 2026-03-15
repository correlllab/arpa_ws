import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    """OpaqueFunction lets us read the params_file at launch time and extract values."""

    params_file = LaunchConfiguration("params_file").perform(context)

    # Defaults
    topics_glob   = LaunchConfiguration("topics_glob").perform(context)
    services_glob = LaunchConfiguration("services_glob").perform(context)
    params_glob   = LaunchConfiguration("params_glob").perform(context)
    actions_glob  = LaunchConfiguration("actions_glob").perform(context)

    # Load YAML and override defaults if keys are present
    if params_file:
        with open(params_file, "r") as f:
            yaml_params = yaml.safe_load(f)

        # Support both flat and ros__parameters-scoped YAML
        ros_params = (
            yaml_params
            .get("rosbridge_websocket", {})
            .get("ros__parameters", yaml_params)  # fallback to flat
        )

        topics_glob   = ros_params.get("topics_glob",   topics_glob)
        services_glob = ros_params.get("services_glob", services_glob)
        params_glob   = ros_params.get("params_glob",   params_glob)
        actions_glob  = ros_params.get("actions_glob",  actions_glob)
        
        rosbridge_node = Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            output="screen",
            parameters=[{
                "port":                            LaunchConfiguration("port"),
                "address":                         LaunchConfiguration("address"),
                "topics_glob":                     str(topics_glob),
                "services_glob":                   str(services_glob),
                "params_glob":                     str(params_glob),
                "max_message_size":                10000000,
                "use_compression":                 False,
                "send_action_goals_in_new_thread": True,
                "call_services_in_new_thread":     False,
                "default_call_service_timeout":    120.0,
                "retry_startup_delay":             10.0,
                "fragment_timeout":                600,
                "delay_between_messages":          5.0,
                "unregister_timeout":              120.0,
            }]
        )

        rosapi_node = Node(
            package="rosapi",
            executable="rosapi_node",
            name="rosapi",
            parameters=[{
                "topics_glob":   str(topics_glob),
                "services_glob": str(services_glob),
                "params_glob":   str(params_glob),
                "params_timeout": 5.0,
            }]
        )

    log_info = LogInfo(
        msg=[
            "\n  Starting Rosbridge WebSocket Server"
            "\n  ├─ Port:          ", LaunchConfiguration("port"),
            "\n  ├─ Address:       ", LaunchConfiguration("address"),
            "\n  ├─ topics_glob:   ", str(topics_glob),
            "\n  ├─ services_glob: ", str(services_glob),
            "\n  ├─ params_glob:   ", str(params_glob),
            "\n  └─ actions_glob:  ", str(actions_glob),
        ]
    )

    return [log_info, rosbridge_node, rosapi_node]


def generate_launch_description():

    port_arg = DeclareLaunchArgument(
        "port", default_value="9090",
        description="Port for rosbridge websocket server"
    )
    address_arg = DeclareLaunchArgument(
        "address", default_value="",
        description="Address to bind (empty = all interfaces)"
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file", default_value="",
        description="Path to a YAML file with topics_glob, services_glob, params_glob, actions_glob"
    )

    # These can still be overridden directly on the CLI — YAML values take precedence if file is provided
    topics_glob_arg   = DeclareLaunchArgument("topics_glob",   default_value="")
    services_glob_arg = DeclareLaunchArgument("services_glob", default_value="")
    params_glob_arg   = DeclareLaunchArgument("params_glob",   default_value="")
    actions_glob_arg  = DeclareLaunchArgument("actions_glob",  default_value="")

    return LaunchDescription([
        port_arg,
        address_arg,
        params_file_arg,
        topics_glob_arg,
        services_glob_arg,
        params_glob_arg,
        actions_glob_arg,
        OpaqueFunction(function=launch_setup),
    ])