#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_simulation_gz"), "/launch/ur_sim_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type": "ur16e",
            "safety_limits": "true",

            # UR control parameters
            "runtime_config_package": "ur_simulation_gz",
            "controllers_file": "ur_controllers.yaml",

            # UR description (URDF/XACRO)
            "description_package": "ur_description",
            "description_file": "ur.urdf.xacro",

            # MoveIt configuration
            "moveit_config_package": "ur_moveit_config",
            "moveit_config_file": "ur.srdf.xacro",

            # Joint prefix (usually empty)
            "prefix": "",
        }.items(),
    )

    ur_manipulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_manipulation"), "/launch/motion_control.launch.py"]
        ),
        launch_arguments={
            "use_sim_time": "true"
        }.items(),
    )

    # ------------------------------------------------------------
    # 3. ARPA GUI
    # Package: arpa_gui
    # File:    arpa_gui.launch.py
    # ------------------------------------------------------------
    arpa_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("arpa_gui"), "/launch/arpa_gui.launch.py"]
        ),
        launch_arguments={
            "use_sim_time": "true"
        }.items(),
    )

    return LaunchDescription([bringup_launch, ur_manipulation, arpa_gui])