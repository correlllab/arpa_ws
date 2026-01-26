import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # ARPA Launch Files
    ur_sim_pkg_share = get_package_share_directory("ur_simulation_gazebo")
    arpa_gui_pkg_share = get_package_share_directory("arpa_gui")

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
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    # Additional xacro arguments
    transmission_hw_interface = LaunchConfiguration("transmission_hw_interface")
    headless_mode = LaunchConfiguration("headless_mode")
    robot_ip = LaunchConfiguration("robot_ip")
    script_filename = LaunchConfiguration("script_filename")
    output_recipe_filename = LaunchConfiguration("output_recipe_filename")
    input_recipe_filename = LaunchConfiguration("input_recipe_filename")
    reverse_ip = LaunchConfiguration("reverse_ip")
    script_command_port = LaunchConfiguration("script_command_port")
    reverse_port = LaunchConfiguration("reverse_port")
    script_sender_port = LaunchConfiguration("script_sender_port")
    trajectory_port = LaunchConfiguration("trajectory_port")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_voltage = LaunchConfiguration("tool_voltage")
    tool_parity = LaunchConfiguration("tool_parity")
    tool_baud_rate = LaunchConfiguration("tool_baud_rate")
    tool_stop_bits = LaunchConfiguration("tool_stop_bits")
    tool_rx_idle_chars = LaunchConfiguration("tool_rx_idle_chars")
    tool_tx_idle_chars = LaunchConfiguration("tool_tx_idle_chars")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    sim_ignition = LaunchConfiguration("sim_ignition")

    # Print all configuration values
    print("=" * 80)
    print("ARPA SIM LAUNCH CONFIGURATION")
    print("=" * 80)
    print(f"  ur_type:                  {ur_type.perform(context)}")
    print(f"  safety_limits:            {safety_limits.perform(context)}")
    print(f"  safety_pos_margin:        {safety_pos_margin.perform(context)}")
    print(f"  safety_k_position:        {safety_k_position.perform(context)}")
    print("-" * 80)
    print("General arguments:")
    print(f"  runtime_config_package:   {runtime_config_package.perform(context)}")
    print(f"  controllers_file:         {controllers_file.perform(context)}")
    print(f"  description_package:      {description_package.perform(context)}")
    print(f"  description_file:         {description_file.perform(context)}")
    print(f"  moveit_config_package:    {moveit_config_package.perform(context)}")
    print(f"  moveit_config_file:       {moveit_config_file.perform(context)}")
    print(f"  prefix:                   {prefix.perform(context)}")
    print(f"  initial_positions_file:   {initial_positions_file.perform(context)}")
    print(f"  start_joint_controller:   {start_joint_controller.perform(context)}")
    print(f"  initial_joint_controller: {initial_joint_controller.perform(context)}")
    print(f"  launch_rviz:              {launch_rviz.perform(context)}")
    print(f"  gazebo_gui:               {gazebo_gui.perform(context)}")
    print("-" * 80)
    print("Hardware/Simulation arguments:")
    print(f"  use_fake_hardware:        {use_fake_hardware.perform(context)}")
    print(f"  fake_sensor_commands:     {fake_sensor_commands.perform(context)}")
    print(f"  sim_gazebo:               {sim_gazebo.perform(context)}")
    print(f"  sim_ignition:             {sim_ignition.perform(context)}")
    print(f"  headless_mode:            {headless_mode.perform(context)}")
    print("-" * 80)
    print("Robot communication arguments:")
    print(f"  robot_ip:                 {robot_ip.perform(context)}")
    print(f"  reverse_ip:               {reverse_ip.perform(context)}")
    print(f"  script_command_port:      {script_command_port.perform(context)}")
    print(f"  reverse_port:             {reverse_port.perform(context)}")
    print(f"  script_sender_port:       {script_sender_port.perform(context)}")
    print(f"  trajectory_port:          {trajectory_port.perform(context)}")
    print("-" * 80)
    print("Script/Recipe arguments:")
    print(f"  script_filename:          {script_filename.perform(context)}")
    print(f"  output_recipe_filename:   {output_recipe_filename.perform(context)}")
    print(f"  input_recipe_filename:    {input_recipe_filename.perform(context)}")
    print(f"  transmission_hw_interface:{transmission_hw_interface.perform(context)}")
    print("-" * 80)
    print("Tool communication arguments:")
    print(f"  use_tool_communication:   {use_tool_communication.perform(context)}")
    print(f"  tool_voltage:             {tool_voltage.perform(context)}")
    print(f"  tool_parity:              {tool_parity.perform(context)}")
    print(f"  tool_baud_rate:           {tool_baud_rate.perform(context)}")
    print(f"  tool_stop_bits:           {tool_stop_bits.perform(context)}")
    print(f"  tool_rx_idle_chars:       {tool_rx_idle_chars.perform(context)}")
    print(f"  tool_tx_idle_chars:       {tool_tx_idle_chars.perform(context)}")
    print(f"  tool_device_name:         {tool_device_name.perform(context)}")
    print(f"  tool_tcp_port:            {tool_tcp_port.perform(context)}")
    print("=" * 80)

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("ur_simulation_gazebo") + "/launch/ur_sim_control.launch.py"
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "safety_pos_margin": safety_pos_margin,
            "safety_k_position": safety_k_position,
            "runtime_config_package": runtime_config_package,
            "controllers_file": controllers_file,
            "description_package": description_package,
            "description_file": description_file,
            "prefix": prefix,
            "launch_rviz": "false",
            "initial_positions_file": initial_positions_file,
            "start_joint_controller": start_joint_controller,
            "initial_joint_controller": initial_joint_controller,
        }.items(),
    )

    arpa_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("arpa_moveit_config") + "/launch/arpa_move_group.launch.py"
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "safety_pos_margin": safety_pos_margin,
            "safety_k_position": safety_k_position,
            "description_package": description_package,
            "description_file": description_file,
            "moveit_config_package": moveit_config_package,
            "moveit_config_file": moveit_config_file,
            "prefix": prefix,
            "use_sim_time": "true",
            "launch_rviz": "true",
            # Additional xacro arguments
            "transmission_hw_interface": transmission_hw_interface,
            "headless_mode": headless_mode,
            "robot_ip": robot_ip,
            "script_filename": script_filename,
            "output_recipe_filename": output_recipe_filename,
            "input_recipe_filename": input_recipe_filename,
            "reverse_ip": reverse_ip,
            "script_command_port": script_command_port,
            "reverse_port": reverse_port,
            "script_sender_port": script_sender_port,
            "trajectory_port": trajectory_port,
            "use_tool_communication": use_tool_communication,
            "tool_voltage": tool_voltage,
            "tool_parity": tool_parity,
            "tool_baud_rate": tool_baud_rate,
            "tool_stop_bits": tool_stop_bits,
            "tool_rx_idle_chars": tool_rx_idle_chars,
            "tool_tx_idle_chars": tool_tx_idle_chars,
            "tool_device_name": tool_device_name,
            "tool_tcp_port": tool_tcp_port,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "sim_gazebo": sim_gazebo,
            "sim_ignition": sim_ignition,
            "initial_positions_file": initial_positions_file,
        }.items(),
    )

    arpa_motion_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("arpa_control") + "/launch/arpa_motion_control.launch.py"
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "safety_pos_margin": safety_pos_margin,
            "safety_k_position": safety_k_position,
            "description_package": description_package,
            "description_file": description_file,
            "moveit_config_package": moveit_config_package,
            "moveit_config_file": moveit_config_file,
            "prefix": prefix,
            "use_sim_time": "true",
            # Additional xacro arguments
            "transmission_hw_interface": transmission_hw_interface,
            "headless_mode": headless_mode,
            "robot_ip": robot_ip,
            "script_filename": script_filename,
            "output_recipe_filename": output_recipe_filename,
            "input_recipe_filename": input_recipe_filename,
            "reverse_ip": reverse_ip,
            "script_command_port": script_command_port,
            "reverse_port": reverse_port,
            "script_sender_port": script_sender_port,
            "trajectory_port": trajectory_port,
            "use_tool_communication": use_tool_communication,
            "tool_voltage": tool_voltage,
            "tool_parity": tool_parity,
            "tool_baud_rate": tool_baud_rate,
            "tool_stop_bits": tool_stop_bits,
            "tool_rx_idle_chars": tool_rx_idle_chars,
            "tool_tx_idle_chars": tool_tx_idle_chars,
            "tool_device_name": tool_device_name,
            "tool_tcp_port": tool_tcp_port,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "sim_gazebo": sim_gazebo,
            "sim_ignition": sim_ignition,
            "initial_positions_file": initial_positions_file,
        }.items(),
    )

    arpa_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("arpa_gui") + "/launch/arpa_gui.launch.py"
        )
    )

    static_tf_world_to_floor = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_floor",
        arguments=["0", "0", "0", "0", "0", "0", "world", "floor_link"]
    )

    return [
        ur_control_launch,
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
            default_value="arpa_moveit_config",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="arpa_controllers.yaml",
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
            default_value=os.path.join(
                get_package_share_directory("arpa_moveit_config"),
                "config",
                "arpa_initial_positions.yaml",
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
    # Additional xacro arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "transmission_hw_interface",
            default_value="",
            description="Transmission hardware interface.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot bringup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="0.0.0.0",
            description="IP address of the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_filename",
            default_value="",
            description="URScript filename for ros_control.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "output_recipe_filename",
            default_value="",
            description="RTDE output recipe filename.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "input_recipe_filename",
            default_value="",
            description="RTDE input recipe filename.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_ip",
            default_value="0.0.0.0",
            description="IP address for reverse communication.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_command_port",
            default_value="50004",
            description="Port for script commands.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_port",
            default_value="50001",
            description="Port for reverse communication.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_sender_port",
            default_value="50002",
            description="Port for script sender.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "trajectory_port",
            default_value="50003",
            description="Port for trajectory communication.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="false",
            description="Enable tool communication.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_voltage",
            default_value="0",
            description="Tool voltage setting.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_parity",
            default_value="0",
            description="Tool parity setting.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_baud_rate",
            default_value="115200",
            description="Tool baud rate.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_stop_bits",
            default_value="1",
            description="Tool stop bits.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_rx_idle_chars",
            default_value="1.5",
            description="Tool RX idle chars.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tx_idle_chars",
            default_value="3.5",
            description="Tool TX idle chars.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_device_name",
            default_value="/tmp/ttyUR",
            description="Tool device name.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="54321",
            description="Tool TCP port.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware for simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake sensor commands.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value="true",
            description="Use Gazebo simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="false",
            description="Use Ignition simulation.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])