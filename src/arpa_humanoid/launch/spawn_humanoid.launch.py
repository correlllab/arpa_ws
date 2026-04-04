"""Spawn H12 (correlllab h12_ros2_model) or legacy G1 in Gazebo; teleport-only, static physics."""

import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _inject_gazebo_static(urdf_xml: str) -> str:
    """Insert Gazebo static model (no gravity / dynamics) before closing robot tag."""
    if "<gazebo>" in urdf_xml and "static" in urdf_xml:
        return urdf_xml
    block = (
        "  <!-- arpa_humanoid: static in Gazebo — teleport only, no physics -->\n"
        "  <gazebo>\n"
        "    <static>true</static>\n"
        "  </gazebo>\n"
    )
    if "</robot>" in urdf_xml:
        return urdf_xml.replace("</robot>", block + "</robot>")
    return urdf_xml + "\n" + block


def launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration("humanoid_model").perform(context).lower().strip()

    if model == "g1":
        g1_share = get_package_share_directory("g1_description")
        urdf_path = os.path.join(g1_share, "urdf", "g1_23dof.urdf")
        entity_name = "unitree_g1"
        tf_child = "g1_base"
        collision_id = "unitree_g1_bbox"
    else:
        # Default: H12 / Unitree H1_2 mesh from correlllab
        h12_share = get_package_share_directory("h12_ros2_model")
        urdf_path = os.path.join(h12_share, "assets", "h1_2", "h1_2_ros.urdf")
        entity_name = "h12_humanoid"
        tf_child = "pelvis"
        collision_id = "h12_humanoid_bbox"

    with open(urdf_path, "r") as f:
        robot_description_content = _inject_gazebo_static(f.read())

    root = ET.fromstring(robot_description_content)
    joint_names = []
    for joint in root.findall("joint"):
        joint_type = joint.attrib.get("type", "")
        if joint_type not in ("fixed", "floating"):
            name = joint.attrib.get("name")
            if name:
                joint_names.append(name)

    spawn_x = LaunchConfiguration("humanoid_x").perform(context)
    spawn_y = LaunchConfiguration("humanoid_y").perform(context)
    spawn_z = LaunchConfiguration("humanoid_z").perform(context)
    spawn_yaw = LaunchConfiguration("humanoid_yaw").perform(context)

    fd, tmp_path = tempfile.mkstemp(prefix="arpa_humanoid_", suffix=".urdf")
    os.close(fd)
    with open(tmp_path, "w") as f:
        f.write(robot_description_content)

    # Separate prefixed TF tree for RViz visualization only. This avoids
    # conflicting with the unprefixed floor_link->pelvis transform used by the GUI.
    humanoid_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="humanoid_robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_content,
                "use_sim_time": True,
                "frame_prefix": "humanoid_visual/",
            },
        ],
        remappings=[
            ("/joint_states", "/humanoid/joint_states"),
            ("robot_description", "/humanoid/robot_description"),
        ],
        output="screen",
    )

    humanoid_jsp = Node(
        package="arpa_humanoid",
        executable="zero_joint_state_publisher.py",
        name="humanoid_joint_state_publisher",
        parameters=[{
            "use_sim_time": True,
            "joint_names_csv": ",".join(joint_names),
            "topic_name": "/humanoid/joint_states",
        }],
        output="screen",
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_humanoid",
        arguments=[
            "-entity", entity_name,
            "-file", tmp_path,
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
            "-Y", spawn_yaw,
        ],
        output="screen",
    )

    teleport_node = Node(
        package="arpa_humanoid",
        executable="humanoid_teleport_node.py",
        name="humanoid_teleport_node",
        parameters=[{
            "use_sim_time": True,
            "initial_x": float(spawn_x),
            "initial_y": float(spawn_y),
            "initial_z": float(spawn_z),
            "initial_yaw": float(spawn_yaw),
            "model_name": entity_name,
            "tf_child_frame": tf_child,
            "collision_object_id": collision_id,
            "publish_collision_box": False,
        }],
        output="screen",
    )

    return [humanoid_rsp, humanoid_jsp, spawn, teleport_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "humanoid_model",
            default_value="h12",
            description="Robot description: h12 (correlllab h12_ros2_model) or g1 (g1_description).",
        ),
        DeclareLaunchArgument("humanoid_x", default_value="0.0",
                              description="Spawn X (m)"),
        DeclareLaunchArgument("humanoid_y", default_value="1.3",
                              description="Spawn Y (m)"),
        DeclareLaunchArgument("humanoid_z", default_value="1.00",
                              description="Spawn Z (m)"),
        DeclareLaunchArgument("humanoid_yaw", default_value="-1.5708",
                              description="Spawn yaw (rad)"),
        OpaqueFunction(function=launch_setup),
    ])
