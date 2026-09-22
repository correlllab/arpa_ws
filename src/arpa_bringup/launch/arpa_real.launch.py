from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction, DeclareLaunchArgument, TimerAction

from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def launch_setup(context, *args, **kwargs):
    arpa_bringup_pkg_share = FindPackageShare("arpa_bringup").find("arpa_bringup")
    ur_robot_driver_pkg_share = FindPackageShare("ur_robot_driver").find("ur_robot_driver")
    arpa_moveit_config_pkg_share = FindPackageShare("arpa_moveit_config").find("arpa_moveit_config")
    arpa_gui_pkg_share = FindPackageShare("arpa_gui").find("arpa_gui")
    arpa_depth_pkg_share = FindPackageShare("cl_realsense").find("cl_realsense")

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    corridor_constraint = LaunchConfiguration("corridor_constraint")
    orientation_constraint = LaunchConfiguration("orientation_constraint")
    analytical_ik = LaunchConfiguration("analytical_ik")
    optimize_path = LaunchConfiguration("optimize_path")
    cost_w_joint = LaunchConfiguration("cost_w_joint")
    cost_w_proximity = LaunchConfiguration("cost_w_proximity")
    cost_w_area = LaunchConfiguration("cost_w_area")
    kdl_random_restart_count = LaunchConfiguration("kdl_random_restart_count")
    kdl_restart_timeout = LaunchConfiguration("kdl_restart_timeout")
    use_zed = LaunchConfiguration("use_zed")
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
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    prefix = LaunchConfiguration("prefix")
    # Additional xacro arguments
    transmission_hw_interface = LaunchConfiguration("transmission_hw_interface")
    script_filename = LaunchConfiguration("script_filename")
    output_recipe_filename = LaunchConfiguration("output_recipe_filename")
    input_recipe_filename = LaunchConfiguration("input_recipe_filename")
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    sim_ignition = LaunchConfiguration("sim_ignition")
    initial_positions_file = LaunchConfiguration("initial_positions_file")

    # Print all configuration values
    print("=" * 80)
    print("ARPA REAL LAUNCH CONFIGURATION")
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
    print(f"  initial_joint_controller: {initial_joint_controller.perform(context)}")
    print(f"  activate_joint_controller:{activate_joint_controller.perform(context)}")
    print(f"  launch_rviz:              {launch_rviz.perform(context)}")
    print(f"  launch_dashboard_client:  {launch_dashboard_client.perform(context)}")
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
    print(f"  initial_positions_file:   {initial_positions_file.perform(context)}")
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

    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [arpa_bringup_pkg_share, "/launch/arpa_control.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "safety_limits": safety_limits,
            "safety_pos_margin": safety_pos_margin,
            "safety_k_position": safety_k_position,
            "runtime_config_package": runtime_config_package,
            "controllers_file": controllers_file,
            "description_package": "arpa_description",
            "description_file": "arpa_system.urdf.xacro",
            "kinematics_params_file": kinematics_params_file,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "headless_mode": headless_mode,
            "controller_spawner_timeout": controller_spawner_timeout,
            "initial_joint_controller": initial_joint_controller,
            "activate_joint_controller": activate_joint_controller,
            "launch_rviz": "false",
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
            "tf_prefix": prefix,
        }.items()
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
            "use_sim_time": "false",
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

    arpa_moveit_launch_delayed = TimerAction(
        period=5.0,
        actions=[arpa_moveit_launch]
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
            "use_sim_time": "false",
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
            "corridor_constraint": corridor_constraint,
            "orientation_constraint": orientation_constraint,
            "analytical_ik": analytical_ik,
            "optimize_path": optimize_path,
            "cost_w_joint": cost_w_joint,
            "cost_w_proximity": cost_w_proximity,
            "cost_w_area": cost_w_area,
            "kdl_random_restart_count": kdl_random_restart_count,
            "kdl_restart_timeout": kdl_restart_timeout,
        }.items(),
    )

    arpa_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [arpa_gui_pkg_share, "/launch/arpa_gui.launch.py"]
        )
    )

    arpa_depth = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [arpa_depth_pkg_share, "/launch/ur16e_rs_cams.launch.py"]
        )
    )

    # ZED camera driver — brought up the same way as the RealSense (arpa_depth, above):
    # an IncludeLaunchDescription of the camera's own launch file. The ZED wrapper ships
    # zed_camera.launch.py; we point it at the first-gen ZED (camera_model:=zed) and name it
    # 'zed' so its base frame is zed_camera_link, matching the static_zed_cam_tf mount below.
    # Gated behind use_zed (default false): the ZED SDK + zed_wrapper build are still pending,
    # and launching an unbuilt package would crash the whole bringup. Flip use_zed:=true once built.
    zed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("zed_wrapper"), "/launch/zed_camera.launch.py"]
        ),
        launch_arguments={
            "camera_model": "zed",
            "camera_name": "zed",
        }.items(),
    )

    ur_rest_api = Node(
        package="ur16e_rest",
        executable="ur16e_rest_api_node"
    )

    ethernet_motor_interface_node = Node(
        package="arpa_ethernet_motor",
        executable="motor_node"
    )

    static_tf_world_to_floor = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_floor",
        arguments=["0", "0", "0", "0", "0", "0", "world", "floor_link"]
    )

    vision_node = Node(
        package="arpa_vision",
        executable="vision_node",
        name="arpa_vision_node"
    )

    # record_images_node = Node(
    #     package="arpa_vision",
    #     executable="record_images",
    #     name="record_images_node"
    # )

    # recorded_poses_publisher = Node(
    #     package="arpa_helper_tools",
    #     executable="record_poses.py",
    #     name="recorded_poses_publisher"
    # )

    scan_battery_server = Node(
        package="arpa_helper_tools",
        executable="scan_battery_action_server.py",
        name="scan_battery_server"
    )

    vla_data_capture = Node(
        package="arpa_helper_tools",
        executable="vla_data_collection_node.py",
        name="vla_data_collection_node"
    )

    vision_node_delayed = TimerAction(
        period=10.0,
        actions=[vision_node]
    )

    test_static_tf_ratchet_attatchemnt = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_ratchet_attachment",
        arguments=["-0.0018", "-0.1488", "0.0922", "0", "0", "-3.14159", "wrist_3_link", "test_ratchet_attachment"]
    )
    test_static_tf_ratchet_ee = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_ratchet_ee",
        arguments=["0", "0", "-0.165", "0", "0", "-3.14159", "test_ratchet_attachment", "test_ratchet_extension_link"]
    )

    # ZED camera mount: mirror of the RealSense ee_cam mount onto the OPPOSITE (+Y) face
    # of the tool, looking DOWN PARALLEL to the RealSense.
    #
    # Source it mirrors: cl_realsense/launch/ur16e_rs_cams.launch.py -> static_ee_cam_tf
    #   RealSense: tool0 -> ee_cam_link  xyz (0.0338, -0.1612, 0.0497)  quat (0.7090, 0.0043, 0.7052, 0.0007)
    #
    # How the mirror was derived:
    #   - Flip ONLY the Y translation (-0.1612 -> +0.1612) to move to the other side of the box.
    #   - Keep the orientation IDENTICAL (NOT reflected). A true geometric mirror would flip
    #     handedness and point the ZED anti-parallel; copying the quaternion keeps both optical
    #     +Z axes pointing the same way down the tool0 +X (nut-runner) axis = genuinely parallel.
    #
    # !!! STARTING ESTIMATE, NOT A CALIBRATION !!!
    #   - The first-gen ZED body is ~2.5x the RealSense, so |Y| likely needs to GROW outward for
    #     clearance against tool_holder_link / the workpiece. Verify physical clearance.
    #   - The ZED needs its OWN hand-eye calibration; replace these numbers once calibrated.
    #   - Child frame 'zed_camera_link' must match the zed_wrapper base frame: launch the ZED node
    #     with camera_name:=zed (camera_model:=zed for the first-gen ZED) so the prefix lines up,
    #     and disable the wrapper's positional-tracking TF (pos_tracking.pos_tracking_enabled:=false)
    #     so it does not try to re-parent zed_camera_link and fight this static transform.
    static_zed_cam_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_zed_cam_tf",
        arguments=[
            "0.033838200335085646", "0.16124934095715254", "0.04973827839776546",
            "0.7090084119721195", "0.004283455595113997", "0.7051866592991314", "0.000706616917886843",
            "tool0",
            "zed_camera_link",
        ]
    )

    rosbridge_mcp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("arpa_bringup") + "/launch/rosbridge_mcp.launch.py"
        ),
        launch_arguments={
            "port":         "9090",
            "address":      "",
            "params_file":  os.path.join(get_package_share_directory('arpa_bringup'), 'config', 'rosbridge_params.yaml'),
        }.items(),
    )

    to_return = [
        ur_driver,
        ur_rest_api,
        # arpa_moveit_launch,
        arpa_moveit_launch_delayed,
        arpa_motion_control,
        # arpa_gui,
        arpa_depth,
        ethernet_motor_interface_node,
        static_tf_world_to_floor,
        vision_node,
        # vision_node_delayed,
        # record_images_node,
        # recorded_poses_publisher
        test_static_tf_ratchet_attatchemnt,
        test_static_tf_ratchet_ee,
        static_zed_cam_tf,
        # rosbridge_mcp,
        # scan_battery_server,
        vla_data_capture
    ]

    # Only bring up the ZED driver when explicitly enabled (use_zed:=true). Kept out of the
    # default set so arpa_real keeps working before the ZED SDK is installed and zed_wrapper built.
    if context.perform_substitution(use_zed).lower() == "true":
        to_return.append(zed_camera)

    return to_return


def generate_launch_description():
    print("HELLO GENERATE LAUNCH DESCRIPTION ARPA REAL")
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
            default_value="192.168.1.5"
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
            description='Package with the controller\'s configuration in "config" folder. '
            "Usually the argument is not set, it enables use of a custom setup.",
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
            description="Description package with robot URDF/XACRO files.",
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
            "kinematics_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("arpa_description"),
                    "config/ur16e",
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
            default_value="30",
            description="Timeout used when spawning controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            choices=[
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
                "freedrive_mode_controller",
                "passthrough_trajectory_controller",
                "parker_linear_actuator"
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
            description="Parity configuration for serial communication.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_baud_rate",
            default_value="115200",
            description="Baud rate configuration for serial communication.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_stop_bits",
            default_value="1",
            description="Stop bits configuration for serial communication.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_rx_idle_chars",
            default_value="1.5",
            description="RX idle chars configuration for serial communication.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tx_idle_chars",
            default_value="3.5",
            description="TX idle chars configuration for serial communication.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_device_name",
            default_value="/tmp/ttyUR",
            description="File descriptor for the tool communication device.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="54321",
            description="Remote port for bridging the tool's serial device.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_voltage",
            default_value="0",
            description="Tool voltage that will be setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_ip",
            default_value="0.0.0.0",
            description="IP for the robot controller to communicate back to the driver.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_command_port",
            default_value="50004",
            description="Port for URScript commands to the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_port",
            default_value="50001",
            description="Port for cyclic instructions from driver to robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_sender_port",
            default_value="50002",
            description="Port for querying the external_control URScript.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "trajectory_port",
            default_value="50003",
            description="Port for trajectory control.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="arpa_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files.",
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
            description="Prefix of the joint names, useful for multi-robot setup.",
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
            "sim_gazebo",
            default_value="false",
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value="",
            description="Initial positions file for simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "corridor_constraint",
            default_value="false",
            description="If true, constrain RRT planning to a corridor between current EE and target. Set to false for benchmark or to allow convoluted paths.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "orientation_constraint",
            default_value="false",
            description="If true, also constrain end-effector orientation along the corridor path. Set to false to constrain position only.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "analytical_ik",
            default_value="false",
            description="If true, generate IK seeds with the closed-form UR16e analytical solver "
            "(with automatic KDL fallback). If false (default), use the original actuator-offset + KDL sweep.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "optimize_path",
            default_value="false",
            description="If true, plan with RRTstar (path-length optimized) instead of RRTConnect.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cost_w_joint",
            default_value="1.0",
            description="Analytical-IK seed ranking weight for normalized joint-distance cost (analytical_ik=true).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cost_w_proximity",
            default_value="1.0",
            description="Analytical-IK seed ranking weight for normalized wrist-to-actuator proximity penalty.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cost_w_area",
            default_value="1.0",
            description="Analytical-IK seed ranking weight for normalized arm-triangle-area (near-singularity) penalty.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kdl_random_restart_count",
            default_value="1",
            description="KDL fallback restarts when analytical IK finds nothing: 1 = single current-state seed; >1 adds random-restart seeds.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kdl_restart_timeout",
            default_value="0.05",
            description="Per-attempt setFromIK timeout (s) for KDL fallback random restarts.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_zed",
            default_value="false",
            description="Launch the ZED camera driver (zed_wrapper, camera_model:=zed) alongside the RealSense. "
            "Requires the ZED SDK installed and zed_wrapper built. Default false until that is set up.",
        )
    )


    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
