import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mobman_bt")
    default_config = os.path.join(pkg_share, "config", "fetch_bt.yaml")

    config_file = LaunchConfiguration("config_file")
    use_mock_pick = LaunchConfiguration("use_mock_pick")
    use_speech = LaunchConfiguration("use_speech")

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("use_mock_pick", default_value="false"),
        DeclareLaunchArgument("use_speech", default_value="false"),

        Node(
            package="mobman_bt",
            executable="fetch_bt_node.py",
            name="fetch_bt_node",
            output="screen",
            parameters=[config_file],
        ),

        Node(
            package="mobman_bt",
            executable="keyboard_voice_node.py",
            name="keyboard_voice_node",
            output="screen",
        ),

        Node(
            package="mobman_bt",
            executable="mock_pick_service.py",
            name="mock_pick_service",
            output="screen",
            condition=IfCondition(use_mock_pick),
        ),

        Node(
            package="mobman_bt",
            executable="speech_voice_node.py",
            name="speech_voice_node",
            output="screen",
            condition=IfCondition(use_speech),
        ),
    ])
