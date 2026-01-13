from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable

def launch_setup(context, *args, **kwargs):
    # ARPA Launch Files
    arpa_bringup_pkg_share = FindPackageShare("arpa_bringup").find("arpa_bringup")
    ur_sim_pkg_share = FindPackageShare("ur_simulation_gazebo").find("ur_simulation_gazebo")
    arpa_moveit_config_pkg_share = FindPackageShare("arpa_moveit_config").find("arpa_moveit_config")
    arpa_gui_pkg_share = FindPackageShare("arpa_gui").find("arpa_gui")
    arpa_depth_pkg_share = FindPackageShare("cl_realsense").find("cl_realsense")
    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    prefix = LaunchConfiguration("prefix")


    # Initialize Arguments
    # General arguments
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    initial_positions_file_abs = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", initial_positions_file]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "prefix:=",
            prefix,
            " ",
            "sim_gazebo:=true",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
            " ",
            "initial_positions_file:=",
            initial_positions_file_abs,
        ]
    )

    print("safety_limits:=",
            safety_limits.perform(context),
            " ",
            "safety_pos_margin:=",
            safety_pos_margin.perform(context),
            " ",
            "safety_k_position:=",
            safety_k_position.perform(context),
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type.perform(context),
            " ",
            "prefix:=",
            prefix.perform(context),
            " ",
            "sim_gazebo:=true",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers.perform(context),
            " ",
            "initial_positions_file:=",
            initial_positions_file_abs.perform(context)
        )

    robot_description = {"robot_description": robot_description_content}

    arpa_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ur_sim_pkg_share, "/launch/ur_sim_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type":ur_type,
            "safety_limits":safety_limits,
            "runtime_config_package":runtime_config_package,
            "controllers_file":controllers_file,
            "description_package":description_package,
            "description_file":description_file,
            "moveit_config_package":moveit_config_package,
            "moveit_config_file":moveit_config_file,
            "prefix":prefix
        }.items()
    )

    arpa_motion_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("arpa_control"), "/launch", "/arpa_motion_control.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "description_package": description_package,
            "description_file": description_file,
            "moveit_config_package": moveit_config_package,
            "moveit_config_file": moveit_config_file,
            "prefix": prefix,
            "use_sim_time": "true",
            "launch_rviz": "true",
            "use_fake_hardware": "true",  # to change moveit default controller to joint_trajectory_controller
        }.items(),
    )

    arpa_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [arpa_gui_pkg_share, "/launch/arpa_gui.launch.py"]
        )
    )

    static_tf_world_to_floor = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_floor",
        arguments=["0", "0", "0", "0", "0", "0", "world", "floor_link"]
    )

    return [
        arpa_moveit_launch,
        arpa_motion_control,
        arpa_gui,
        static_tf_world_to_floor
    ]


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur7e",
                "ur10",
                "ur12e",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur16e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ur_simulation_gazebo",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="arpa_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="arpa_system.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="arpa_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="arpa_system.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("arpa_moveit_config"),
                    "config",
                    "initial_positions.yaml",
                ]
            ),
            description="YAML file (absolute path) with the robot's initial joint positions.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_gui", default_value="true", description="Start gazebo with GUI?"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])