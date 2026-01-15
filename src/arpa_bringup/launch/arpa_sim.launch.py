import os
from ament_index_python.packages import get_package_share_directory
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def launch_setup(context, *args, **kwargs):

    # ---------------------------------------------------------
    # Launch Arguments
    # ---------------------------------------------------------
    description_pkg = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    moveit_pkg = LaunchConfiguration("moveit_config_package")
    moveit_controllers = LaunchConfiguration("controllers_file")
    initial_positions = LaunchConfiguration("initial_positions_file")

    gui = LaunchConfiguration("gazebo_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")
    start_joint_controller = LaunchConfiguration("start_joint_controller")

    prefix = LaunchConfiguration("prefix")

    print("DEBUG2: description_pkg =", description_pkg.perform(context))
    print("DEBUG2: moveit_pkg =", moveit_pkg.perform(context))

    print("DEBUG3: FindPackageShare(description_pkg) ->",
        FindPackageShare(description_pkg).perform(context))

    print("DEBUG3: FindPackageShare(moveit_pkg) ->",
        FindPackageShare(moveit_pkg).perform(context))
    # ---------------------------------------------------------
    # Absolute paths
    # ---------------------------------------------------------
    controller_yaml = PathJoinSubstitution(
        [FindPackageShare(moveit_pkg), "config", moveit_controllers]
    )

    initial_positions_abs = PathJoinSubstitution(
        [FindPackageShare(moveit_pkg), "config", initial_positions]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_pkg), "rviz", "arpa_visual.rviz"]
    )

    # ---------------------------------------------------------
    # MoveIt Configuration for RViz
    # ---------------------------------------------------------
    moveit_pkg_str = moveit_pkg.perform(context)
    moveit_pkg_share = get_package_share_directory(moveit_pkg_str)
    
    # Load robot_description_semantic (SRDF)
    srdf_file = os.path.join(moveit_pkg_share, "config", "arpa_system.srdf")
    with open(srdf_file, 'r') as f:
        robot_description_semantic_content = f.read()
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    
    # Kinematics file path (use ParameterFile to load it)
    kinematics_file = PathJoinSubstitution(
        [FindPackageShare(moveit_pkg), "config", "kinematics.yaml"]
    )
    
    # Load joint limits
    joint_limits_file = os.path.join(moveit_pkg_share, "config", "joint_limits.yaml")
    with open(joint_limits_file, 'r') as f:
        robot_description_planning = {"robot_description_planning": yaml.safe_load(f)}
    
    # Load OMPL planning config
    ompl_file = os.path.join(get_package_share_directory("ur_moveit_config"), "config", "ompl_planning.yaml")
    with open(ompl_file, 'r') as f:
        ompl_yaml = yaml.safe_load(f)
    
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config["move_group"].update(ompl_yaml)

    # Load controllers config for MoveIt
    controllers_file_path = os.path.join(moveit_pkg_share, "config", "controllers.yaml")
    with open(controllers_file_path, 'r') as f:
        controllers_yaml = yaml.safe_load(f)
    
    # Set default controller for simulation
    controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
    controllers_yaml["joint_trajectory_controller"]["default"] = True
    
    moveit_controllers_config = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": "",
    }

    # ---------------------------------------------------------
    # Build robot_description from your ARPA xacro
    # ---------------------------------------------------------
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare(description_pkg),
            "urdf",
            description_file
        ]),
        " ",
        "sim_gazebo:=true",
        " ",
        "simulation_controllers:=",
        controller_yaml,
    ])

    print(robot_description_content.perform(context))

    robot_description = {"robot_description": robot_description_content.perform(context)}

    # ---------------------------------------------------------
    # Robot State Publisher
    # ---------------------------------------------------------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # ---------------------------------------------------------
    # Joint State Broadcaster + Trajectory Controller
    # ---------------------------------------------------------
    js_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    traj_controller_active = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
        output="screen",
    )

    traj_controller_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
        output="screen",
    )

    # Linear actuator controller
    linear_actuator_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["linear_actuator_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # ---------------------------------------------------------
    # Gazebo Classic
    # ---------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch/gazebo.launch.py"]
        ),
        launch_arguments={"gui": gui}.items(),
    )

    # ---------------------------------------------------------
    # Gazebo Entity Spawn
    # ---------------------------------------------------------
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "arpa_system", "-topic", "robot_description"],
        output="screen",
    )

    # ---------------------------------------------------------
    # Launch RVIZ (after a short delay to ensure everything is ready)
    # ---------------------------------------------------------
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            ParameterFile(kinematics_file, allow_substs=True),
            robot_description_planning,
            ompl_planning_pipeline_config,
            {"use_sim_time": True},
        ],
    )

    # Launch RViz after a 3 second delay to ensure move_group and other nodes are ready
    delay_rviz = TimerAction(
        period=3.0,
        actions=[rviz],
    )

    # ---------------------------------------------------------
    # Launch MoveIt (MoveGroup)
    # ---------------------------------------------------------
    # Convert controller_yaml to string for launch_arguments
    controller_yaml_str = controller_yaml.perform(context)
    
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare(moveit_pkg), "/launch/move_group.launch.py"]
        ),
        launch_arguments={
            "use_sim_time": "true",
            "description_package": description_pkg.perform(context),
            "description_file": description_file.perform(context),
            "moveit_config_package": moveit_pkg.perform(context),
            "simulation_controllers": controller_yaml_str,
            "launch_rviz": "false",  # RViz launched separately in this file
        }.items(),
    )

    print("DEBUG: description_pkg =", description_pkg.perform(context))
    print("DEBUG: description_file =", description_file.perform(context))
    print("DEBUG: moveit_pkg =", moveit_pkg.perform(context))
    print("DEBUG: controllers_file =", moveit_controllers.perform(context))
    print("DEBUG: initial_positions =", initial_positions.perform(context))
    print("DEBUG: gui =", gui.perform(context))
    print("DEBUG: prefix =", prefix.perform(context))


    motion_control = Node(
        package='ur_manipulation',
        executable='motion_control_node',
        output='screen',
        parameters=[
            {'octomap_resolution': 0.01,},
            robot_description,
            robot_description_semantic,
            ParameterFile(kinematics_file, allow_substs=True),
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers_config,
            planning_scene_monitor_parameters,
            {"use_sim_time": True},
            warehouse_ros_config,
        ],
    )

    # ---------------------------------------------------------
    # Static TF: world -> floor_link (required for MoveIt planning)
    # ---------------------------------------------------------
    static_tf_world_to_floor = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_floor",
        arguments=["--frame-id", "world", "--child-frame-id", "floor_link",
                   "--x", "0", "--y", "0", "--z", "0",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1"],
        parameters=[{"use_sim_time": True}],
    )

    # ---------------------------------------------------------
    # Launch ARPA GUI
    # ---------------------------------------------------------
    arpa_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("arpa_gui"), "/launch/arpa_gui.launch.py"]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )


    # Relay node to convert GUI topic to controller command
    linear_actuator_relay = Node(
        package="topic_tools",
        executable="relay",
        name="linear_actuator_relay",
        arguments=[
            "/linear_actuator_joint_position",
            "/linear_actuator_controller/commands"
        ],
        output="screen",
    )

    return [
        gazebo,
        static_tf_world_to_floor,  # Publish TF before robot state publisher
        rsp,
        js_broadcaster,
        traj_controller_active,
        traj_controller_stopped,
        linear_actuator_controller,
        linear_actuator_relay,
        spawn_robot,
        move_group_launch,
        motion_control,
        arpa_gui,
        delay_rviz,  # Launch RViz after 3 second delay
    ]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("description_package", default_value="arpa_description"),
            DeclareLaunchArgument("description_file", default_value="arpa_system.urdf.xacro"),

            DeclareLaunchArgument("moveit_config_package", default_value="arpa_moveit_config"),
            DeclareLaunchArgument("controllers_file", default_value="ros2_controllers.yaml"),
            DeclareLaunchArgument("initial_positions_file", default_value="initial_positions.yaml"),

            DeclareLaunchArgument("prefix", default_value='""'),

            DeclareLaunchArgument("start_joint_controller", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("gazebo_gui", default_value="true"),

            OpaqueFunction(function=launch_setup),
        ]
    )