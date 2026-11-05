from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "mobman",
        package_name="mobman_moveit_config",
    ).to_moveit_configs()

    return LaunchDescription([
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {
                    "use_sim_time": True,
                    "publish_robot_description": True,
                    "publish_robot_description_semantic": True,
                },
            ],
        )
    ])