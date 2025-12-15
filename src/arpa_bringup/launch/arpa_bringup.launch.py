from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("arpa_system", package_name="arpa_moveit_config")
        .to_moveit_configs()
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            "config/ros2_controllers.yaml",
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    delay_broad = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    delay_traj = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur16e_on_gantry_controller"],
    )

    move_group = generate_move_group_launch(moveit_config)

    return LaunchDescription([
        rsp,
        control_node,
        delay_broad,
        delay_traj,
        move_group,
    ])