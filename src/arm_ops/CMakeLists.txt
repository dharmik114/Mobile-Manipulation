cmake_minimum_required(VERSION 3.8)
project(arm_ops)

# REQUIRED
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(goal_pose src/goal_pose.cpp)

ament_target_dependencies(goal_pose
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

install(TARGETS goal_pose
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()