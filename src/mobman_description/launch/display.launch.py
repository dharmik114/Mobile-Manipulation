from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    use_sim_time = True
    
    urdf_path = os.path.join(get_package_share_directory('mobman_description'), 'urdf', 'mobman.urdf.xacro')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    
    rviz_path = os.path.join(get_package_share_directory('mobman_description'), 'rviz', 'model.rviz')

    gazebo_path = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    world_path = os.path.join(get_package_share_directory('mobman_description'), 'world', 'ob_world.sdf')

    bridge_path = os.path.join(get_package_share_directory('mobman_description'), 'config', 'bridge_config.yaml')

    ekf_path = os.path.join(get_package_share_directory('mobman_description'), 'config', 'ekf.yaml')

    pkg_share = get_package_share_directory('mobman_description')

    # Gazebo Sim looks for a folder named 'mobman_description' inside the paths 
    # listed in GZ_SIM_RESOURCE_PATH. Since 'pkg_share' IS the 
    # mobman_description folder, we point to its parent.
    gz_resource_path = os.path.join(pkg_share, '..')
    gz_resource_path = os.path.normpath(gz_resource_path)

    # Use BOTH variable names to be safe (for different Gazebo/Ignition versions)
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[gz_resource_path]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description},
                    {'use_sim_time': use_sim_time}]
    )

    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_path)),
        launch_arguments=[('gz_args', f'-r {world_path}')]
        )                                                          # launches gazebo, (ros2 launch ros_gz_sim gz_sim.launch.py)

    spawn_entity_node = Node(                                      # spawns the robot (spawns the revageclaw)
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=['-entity', 'mobman', '-topic', 'robot_description']
    )

    ros_gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{'config_file': bridge_path}],
        output="screen"
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_path,
                    {'use_sim_time': use_sim_time}]
    )
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["manipulator_moveit_controller"],
    )

    spawn_gripper_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["gripper_moveit_controller"],
    )

    controller_spawners = RegisterEventHandler(
    OnProcessExit(
        target_action=spawn_entity_node,
        on_exit=[
            spawn_jsb,
            spawn_arm_controller,
            spawn_gripper_controller,
        ],
    )
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_path]
    )

    return LaunchDescription([
    set_gz_resource_path,
    gazebo_node, 
    robot_state_publisher_node, 
    # joint_state_publisher_node,           
    spawn_entity_node,       
    ros_gz_bridge_node,      
    ekf_node,
    controller_spawners,
    rviz2_node
    ])