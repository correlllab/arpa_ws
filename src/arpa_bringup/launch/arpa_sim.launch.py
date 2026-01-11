from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    ExecuteProcess,
    SetEnvironmentVariable,
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
    # Launch RVIZ only after JS broadcaster is up
    # ---------------------------------------------------------
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file]
    )

    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=js_broadcaster,
            on_exit=[rviz],
        ),
        condition=IfCondition(launch_rviz),
    )

    # ---------------------------------------------------------
    # Launch MoveIt (MoveGroup)
    # ---------------------------------------------------------
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare(moveit_pkg), "/launch/move_group.launch.py"]
        ),
    )

    # ---------------------------------------------------------
    # Launch ARPA GUI
    # ---------------------------------------------------------
    arpa_gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("arpa_gui"), "/launch/arpa_gui.launch.py"]
        ),
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
            moveit_controllers,
            {"use_sim_time": use_sim_time},
        ],
    )

    arpa_gui_pkg_share = FindPackageShare("arpa_gui").find("arpa_gui")
    arpa_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [arpa_gui_pkg_share, "/launch/arpa_gui.launch.py"]
        )
    )


    return [
        gazebo,
        rsp,
        js_broadcaster,
        traj_controller_active,
        traj_controller_stopped,
        spawn_robot,
        move_group_launch,
        motion_control,
        arpa_gui,
        # delay_rviz
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