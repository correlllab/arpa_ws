from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Use the manual launch file instead of auto-generated one
    # This gives us more control over configuration
    arpa_move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("arpa_moveit_config"), "/launch/arpa_move_group.launch.py"]
        ),
        launch_arguments={
            "use_sim_time": "true",  # We're in simulation
        }.items(),
    )
    
    return LaunchDescription([
        arpa_move_group_launch,
    ])
