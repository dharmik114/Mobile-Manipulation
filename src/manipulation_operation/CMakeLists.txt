cmake_minimum_required(VERSION 3.8)
project(manipulation_operation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(pick_and_place_node src/pickandplace.cpp)

ament_target_dependencies(pick_and_place_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_srvs
)

install(TARGETS pick_and_place_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
