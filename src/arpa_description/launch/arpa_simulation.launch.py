from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    # ------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------
    declared_arguments = [
        DeclareLaunchArgument(
            "use_gui",
            default_value="true",
            description="Launch joint_state_publisher with GUI sliders."
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz2 with preloaded config."
        ),
    ]

    use_gui = LaunchConfiguration("use_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # ------------------------------------------------------------
    # Package paths
    # ------------------------------------------------------------
    arpa_pkg = FindPackageShare("arpa_description")
    urdf_path = PathJoinSubstitution([arpa_pkg, "urdf", "arpa_system.urdf.xacro"])
    rviz_path = PathJoinSubstitution([arpa_pkg, "rviz", "arpa.rviz"])

    # ------------------------------------------------------------
    # Nodes
    # ------------------------------------------------------------

    # 1. Robot State Publisher — reads the generated URDF and publishes TFs
    robot_description_content = Command(["xacro ", urdf_path])
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": ParameterValue(robot_description_content, value_type=str)}],
    )

    # # 2. Joint State Publisher — provides simulated joint states
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher"
    )

    # # 3. RViz — visualize the gantry + UR16e system
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
        condition=None,  # always start; can make conditional if desired
    )

    ur_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_simulation_gazebo"),
                "launch",
                "ur_sim_moveit.launch.py",
            ])
        ),
        launch_arguments={
            "ur_type": "ur16e",
            "description_package": "arpa_description",
            "description_file": "arpa_system.urdf.xacro",
            "origin": "ur_base_link",
            "use_fake_hardware": "true",
            "launch_rviz": "false",
            "runtime_config_package": "ur_simulation_gazebo",
            "controllers_file": "ur_controllers.yaml",
            "moveit_config_package": "ur_moveit_config",
            "moveit_config_file": "ur.srdf.xacro",
            "prefix": "",
        }.items(),
    )

    # ------------------------------------------------------------
    # Return composed launch description
    # ------------------------------------------------------------
    return LaunchDescription(
        declared_arguments + [
            LogInfo(msg="Launching ARPA gantry + inverted UR16e simulation..."),
            robot_state_publisher,
            joint_state_publisher,
            rviz,
            ur_simulation
        ]
    )