import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("super_odometry"),
        "config",
        "campus_map.yaml",
    )

    config_file = LaunchConfiguration("config_file")
    map_directory = LaunchConfiguration("map_directory")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Campus map parameter file",
        ),
        DeclareLaunchArgument(
            "map_directory",
            default_value="/root/ros2_ws/src/SuperOdom/maps",
            description="Base directory for timestamped campus map sessions",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use ROS simulation time",
        ),
        Node(
            package="super_odometry",
            executable="campus_map_node",
            name="campus_map_node",
            output="screen",
            parameters=[
                config_file,
                {
                    "map_directory": map_directory,
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                },
            ],
        ),
    ])
