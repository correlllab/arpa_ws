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

def launch_setup(context, *args, **kwargs):
    print("HELLO LAUNCH SETUP ARPA UR CONTROL")
    arpa_bringup_pkg_share = FindPackageShare("arpa_bringup").find("arpa_bringup")
    ur_moveit_config_pkg_share = FindPackageShare("ur_moveit_config").find("ur_moveit_config")
    arpa_moveit_config_pkg_share = FindPackageShare("arpa_moveit_config").find("arpa_moveit_config")
    ur_manipulation_pkg_share = FindPackageShare("ur_manipulation").find("ur_manipulation")
    arpa_gui_pkg_share = FindPackageShare("arpa_gui").find("arpa_gui")
    arpa_depth_pkg_share = FindPackageShare("cl_realsense").find("cl_realsense")
    moveit_srdf = [FindPackageShare("arpa_moveit_config"), "srdf", "arpa_system.srdf"]
    print("HELLO LAUNCH SETUP ARPA UR CONTROL")

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_parity = LaunchConfiguration("tool_parity")
    tool_baud_rate = LaunchConfiguration("tool_baud_rate")
    tool_stop_bits = LaunchConfiguration("tool_stop_bits")
    tool_rx_idle_chars = LaunchConfiguration("tool_rx_idle_chars")
    tool_tx_idle_chars = LaunchConfiguration("tool_tx_idle_chars")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    tool_voltage = LaunchConfiguration("tool_voltage")
    reverse_ip = LaunchConfiguration("reverse_ip")
    script_command_port = LaunchConfiguration("script_command_port")
    reverse_port = LaunchConfiguration("reverse_port")
    script_sender_port = LaunchConfiguration("script_sender_port")
    trajectory_port = LaunchConfiguration("trajectory_port")
    print("HELLO LAUNCH SETUP ARPA UR CONTRO 323 L")

    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [arpa_bringup_pkg_share, "/launch/arpa_ur_control_jazzy.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "safety_limits": safety_limits,
            "safety_pos_margin": safety_pos_margin,
            "safety_k_position": safety_k_position,
            "runtime_config_package": runtime_config_package,
            "controllers_file": controllers_file,
            "description_package": "arpa_moveit_config",  # override default
            "description_file": "arpa_system.urdf.xacro", # override default
            "kinematics_params_file": kinematics_params_file,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "headless_mode": headless_mode,
            "controller_spawner_timeout": controller_spawner_timeout,
            "initial_joint_controller": initial_joint_controller,
            "activate_joint_controller": activate_joint_controller,
            "launch_rviz": launch_rviz,
            "launch_dashboard_client": launch_dashboard_client,
            "use_tool_communication": use_tool_communication,
            "tool_parity": tool_parity,
            "tool_baud_rate": tool_baud_rate,
            "tool_stop_bits": tool_stop_bits,
            "tool_rx_idle_chars": tool_rx_idle_chars,
            "tool_tx_idle_chars": tool_tx_idle_chars,
            "tool_device_name": tool_device_name,
            "tool_tcp_port": tool_tcp_port,
            "tool_voltage": tool_voltage,
            "reverse_ip": reverse_ip,
            "script_command_port": script_command_port,
            "reverse_port": reverse_port,
            "script_sender_port": script_sender_port,
            "trajectory_port": trajectory_port,
        }.items()
    )
    print("IM OUT")

    # print("YES")
    # moveit_xacro_mappings = {
    #     "ur_type": ur_type,
    #     "robot_ip": robot_ip,
    #     "safety_limits": safety_limits,
    #     "safety_pos_margin": safety_pos_margin,
    #     "safety_k_position": safety_k_position,
    #     "runtime_config_package": runtime_config_package,
    #     "controllers_file": controllers_file,
    #     "description_package": "arpa_moveit_config",  # override default
    #     "description_file": "arpa_system.urdf.xacro", # override default
    #     "kinematics_params_file": kinematics_params_file,
    #     "use_fake_hardware": use_fake_hardware,
    #     "fake_sensor_commands": fake_sensor_commands,
    #     "headless_mode": headless_mode,
    #     "controller_spawner_timeout": controller_spawner_timeout,
    #     "initial_joint_controller": initial_joint_controller,
    #     "activate_joint_controller": activate_joint_controller,
    #     "launch_rviz": launch_rviz,
    #     "launch_dashboard_client": launch_dashboard_client,
    #     "use_tool_communication": use_tool_communication,
    #     "tool_parity": tool_parity,
    #     "tool_baud_rate": tool_baud_rate,
    #     "tool_stop_bits": tool_stop_bits,
    #     "tool_rx_idle_chars": tool_rx_idle_chars,
    #     "tool_tx_idle_chars": tool_tx_idle_chars,
    #     "tool_device_name": tool_device_name,
    #     "tool_tcp_port": tool_tcp_port,
    #     "tool_voltage": tool_voltage,
    #     "reverse_ip": reverse_ip,
    #     "script_command_port": script_command_port,
    #     "reverse_port": reverse_port,
    #     "script_sender_port": script_sender_port,
    #     "trajectory_port": trajectory_port
    # }
    # # Load the robot configuration
    # moveit_config = (
    #     MoveItConfigsBuilder(
    #         "arpa_system", package_name="arpa_moveit_config"
    #     )
    #     .robot_description(file_path=get_package_share_directory("arpa_moveit_config") + "/urdf/arpa_system.urdf.xacro", mappings=moveit_xacro_mappings)
    #     .robot_description_semantic(file_path=get_package_share_directory("arpa_moveit_config") + "/config/arpa_system.srdf")
    #     .robot_description_kinematics(file_path=get_package_share_directory("arpa_moveit_config") + "/config/kinematics.yaml")
    #     .joint_limits(file_path=get_package_share_directory("arpa_moveit_config") + "/config/joint_limits.yaml")
    #     .trajectory_execution(file_path=get_package_share_directory("arpa_moveit_config") + "/config/real_moveit_controllers.yaml")
    #     .moveit_cpp(file_path=get_package_share_directory("arpa_moveit_config") + "/config/moveit_cpp.yaml")
    #     .pilz_cartesian_limits(file_path=get_package_share_directory("arpa_moveit_config") + "/config/pilz_cartesian_limits.yaml")
    #     .to_moveit_configs()
    # )

    # print("MAYBE JUST MAYBE")
    # planning_scene_monitor_parameters = {
    #     'publish_planning_scene': True,
    #     'publish_geometry_updates': True,
    #     'publish_state_updates': True,
    #     'publish_transforms_updates': True,
    #     'publish_robot_description': True,
    #     'publish_robot_description_semantic': True
    #     # "planning_scene_monitor_options": {
    #     #     "name": "planning_scene_monitor",
    #     #     "robot_description": "robot_description",
    #     #     "joint_state_topic": "/joint_states",
    #     #     "attached_collision_object_topic": "/move_group/planning_scene_monitor",
    #     #     "publish_planning_scene_topic": "/move_group/publish_planning_scene",
    #     #     "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
    #     #     "wait_for_initial_state_timeout": 10.0,
    #     # },
    # }

    # move_group_node = Node(
    #     package='moveit_ros_move_group',
    #     executable='move_group', 
    #     output='screen',
    #     parameters=[
    #         moveit_config.to_dict(),
    #         planning_scene_monitor_parameters,
    #         {'use_sim_time': False},
    #     ],
    #     #arguments=['--ros-args', '--log-level', 'DEBUG', '--log-level', 'rcl:=WARN', '--log-level', 'moveit_kinematics_base.kinematics_base:=WARN'],
    # )

    # # rviz with moveit configuration
    # # rviz_config_file = get_package_share_directory("arpa_description") + "/rviz/arap.rviz"
    # # rviz_node = Node(
    # #     package="rviz2",
    # #     executable="rviz2",
    # #     name="rviz2_moveit",
    # #     output="log",
    # #     arguments=["-d", rviz_config_file]
    # # )

    print("HELLO LAUNCH SETUP ARPA UR CONTRO 323 N")
    arpa_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [arpa_bringup_pkg_share, "/launch/arpa_moveit_jazzy.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "safety_limits": safety_limits,
            "safety_pos_margin": safety_pos_margin,
            "safety_k_position": safety_k_position,
            "runtime_config_package": runtime_config_package,
            "controllers_file": controllers_file,
            "description_package": "arpa_moveit_config",  # override default
            "description_file": "arpa_system.urdf.xacro", # override default
            "kinematics_params_file": kinematics_params_file,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "headless_mode": headless_mode,
            "controller_spawner_timeout": controller_spawner_timeout,
            "initial_joint_controller": initial_joint_controller,
            "activate_joint_controller": activate_joint_controller,
            "launch_rviz": launch_rviz,
            "launch_dashboard_client": launch_dashboard_client,
            "use_tool_communication": use_tool_communication,
            "tool_parity": tool_parity,
            "tool_baud_rate": tool_baud_rate,
            "tool_stop_bits": tool_stop_bits,
            "tool_rx_idle_chars": tool_rx_idle_chars,
            "tool_tx_idle_chars": tool_tx_idle_chars,
            "tool_device_name": tool_device_name,
            "tool_tcp_port": tool_tcp_port,
            "tool_voltage": tool_voltage,
            "reverse_ip": reverse_ip,
            "script_command_port": script_command_port,
            "reverse_port": reverse_port,
            "script_sender_port": script_sender_port,
            "trajectory_port": trajectory_port,
        }.items()
    )
    print("HELLO LAUNCH SETUP ARPA UR CONTRO 323 N")


    # arpa_motion = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [ur_manipulation_pkg_share, "/launch/motion_control.launch.py"]
    #     )
    # )

    # arpa_gui = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [arpa_gui_pkg_share, "/launch/arpa_gui.launch.py"]
    #     )
    # )

    # arpa_depth = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [arpa_depth_pkg_share, "/launch/ur16e_rs_cams.launch.py"]
    #     )
    # )

    # ur_rest_api = Node(
    #     package="ur16e_rest",
    #     executable="ur16e_rest_api_node"
    # )

    # ethernet_motor_interface_node = Node(
    #     package="arpa_ethernet_motor",
    #     executable="motor_node"
    # )

    static_tf_world_to_floor = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_floor",
        arguments=["0", "0", "0", "0", "0", "0", "world", "floor_link"]
    )

    return [
        ur_driver,
        # ur_rest_api,
        arpa_moveit_launch,
        # rviz_node,
        # arpa_move_group_launch,
        # arpa_gui,
        # arpa_motion,
        # arpa_depth,
        # ethernet_motor_interface_node,
        static_tf_world_to_floor
    ]


def generate_launch_description():
    print("HELLO GENERATE LAUNCH DESCRIPTION ARPA UR CONTROL")
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur5",
                "ur10",
                "ur3e",
                "ur5e",
                "ur7e",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur8long",
                "ur15",
                "ur18",
                "ur20",
                "ur30",
            ],
            default_value="ur16e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="IP address by which the robot can be reached.",
            default_value="128.138.224.247"
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
            default_value="ur_robot_driver",
            description='Package with the controller\'s configuration in "config" folder. '
            "Usually the argument is not set, it enables use of a custom setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("arpa_moveit_config"), "config", "ur_controllers.yaml"]
            ),
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare(LaunchConfiguration("description_package")),
                    "config",
                    LaunchConfiguration("ur_type"),
                    "default_kinematics.yaml",
                ]
            ),
            description="The calibration configuration of the actual robot used.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. "
            "Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            choices=[
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
                "freedrive_mode_controller",
                "passthrough_trajectory_controller",
            ],
            description="Initially loaded robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_dashboard_client", default_value="true", description="Launch Dashboard Client?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="false",
            description="Only available for e series!",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_parity",
            default_value="0",
            description="Parity configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_baud_rate",
            default_value="115200",
            description="Baud rate configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_stop_bits",
            default_value="1",
            description="Stop bits configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_rx_idle_chars",
            default_value="1.5",
            description="RX idle chars configuration for serial communication. Only effective, "
            "if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tx_idle_chars",
            default_value="3.5",
            description="TX idle chars configuration for serial communication. Only effective, "
            "if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_device_name",
            default_value="/tmp/ttyUR",
            description="File descriptor that will be generated for the tool communication device. "
            "The user has be be allowed to write to this location. "
            "Only effective, if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="54321",
            description="Remote port that will be used for bridging the tool's serial device. "
            "Only effective, if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_voltage",
            default_value="0",  # 0 being a conservative value that won't destroy anything
            description="Tool voltage that will be setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_ip",
            default_value="0.0.0.0",
            description="IP that will be used for the robot controller to communicate back to the driver.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_command_port",
            default_value="50004",
            description="Port that will be opened to forward URScript commands to the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_port",
            default_value="50001",
            description="Port that will be opened to send cyclic instructions from the driver to the robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_sender_port",
            default_value="50002",
            description="The driver will offer an interface to query the external_control URScript on this port.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "trajectory_port",
            default_value="50003",
            description="Port that will be opened for trajectory control.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])