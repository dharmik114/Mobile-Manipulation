import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_keyboard = LaunchConfiguration("use_keyboard")
    use_speech = LaunchConfiguration("use_speech")
    bt_config = LaunchConfiguration("bt_config")

    description_pkg = get_package_share_directory("mobman_description")
    moveit_pkg = get_package_share_directory("mobman_moveit_config")
    navigation_pkg = get_package_share_directory("mobman_navigation")
    bt_pkg = get_package_share_directory("mobman_bt")

    return LaunchDescription([
        DeclareLaunchArgument("use_keyboard", default_value="true"),
        DeclareLaunchArgument("use_speech", default_value="false"),
        DeclareLaunchArgument(
            "bt_config",
            default_value=os.path.join(bt_pkg, "config", "fetch_bt.yaml"),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_pkg, "launch", "display.launch.py")
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation_pkg, "launch", "navigation.launch.py")
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_pkg, "launch", "move_group.launch.py")
            )
        ),

        Node(
            package="mobman_perception",
            executable="object_detection.py",
            name="yolo_3d_detector",
            output="screen",
        ),

        Node(
            package="manipulation_operation",
            executable="pick_and_place_node",
            name="pick_object_service",
            output="screen",
        ),

        Node(
            package="mobman_bt",
            executable="fetch_bt_node.py",
            name="fetch_bt_node",
            output="screen",
            parameters=[bt_config],
        ),

        Node(
            package="mobman_bt",
            executable="keyboard_voice_node.py",
            name="keyboard_voice_node",
            output="screen",
            condition=IfCondition(use_keyboard),
        ),

        Node(
            package="mobman_bt",
            executable="speech_voice_node.py",
            name="speech_voice_node",
            output="screen",
            condition=IfCondition(use_speech),
        ),
    ])
