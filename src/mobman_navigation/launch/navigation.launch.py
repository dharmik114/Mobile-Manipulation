import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav_pkg = get_package_share_directory("mobman_navigation")
    nav2_pkg = get_package_share_directory("nav2_bringup")

    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(nav_pkg, "map", "my_map.yaml"),
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(nav_pkg, "config", "nav2_params.yaml"),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_pkg, "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "map": map_file,
                "params_file": params_file,
                "use_sim_time": use_sim_time,
                "autostart": "true",
            }.items(),
        ),
    ])
