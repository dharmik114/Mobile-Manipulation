#include <memory>
#include <chrono>
#include <map>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "pick_and_place_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("pick_and_place");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]()
              { executor.spin(); })
      .detach();

  moveit::planning_interface::MoveGroupInterface arm_group(node, "manipulator");
  moveit::planning_interface::MoveGroupInterface gripper_group(node, "gripper");

  arm_group.setMaxVelocityScalingFactor(0.5);
  arm_group.setMaxAccelerationScalingFactor(0.5);
  gripper_group.setMaxVelocityScalingFactor(0.3);
  gripper_group.setMaxAccelerationScalingFactor(0.3);

  auto set_gripper = [&](double position)
  {
    gripper_group.setStartStateToCurrentState();
    gripper_group.setJointValueTarget(
        std::map<std::string, double>{
            {"gripper_left_joint", position},
            {"gripper_right_joint", position},
        });

    auto result = gripper_group.move();
    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(logger, "Gripper moved to %.3f", position);
    }
    else
    {
      RCLCPP_ERROR(logger, "Gripper movement failed with code %d", result.val);
    }
    rclcpp::sleep_for(std::chrono::seconds(1));
  };

  RCLCPP_INFO(logger, "--- Initializing Pick and Place Sequence ---");

  RCLCPP_INFO(logger, "Stage 1: Opening Gripper...");
  set_gripper(0.019);

  RCLCPP_INFO(logger, "Stage 2: Moving to Pre-Pick pose...");
  std::vector<double> arm_joint_targets = {0.0, 0.2, 0.2, -0.4}; // joint1, joint2, joint3, joint4
  arm_group.setJointValueTarget(arm_joint_targets);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    arm_group.execute(my_plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(logger, "Stage 3: Closing Gripper");
  set_gripper(-0.010);

  RCLCPP_INFO(logger, "Stage 4: Lifting Object...");
  arm_joint_targets = {0.0, -0.2, 0.4, -0.2};
  arm_group.setJointValueTarget(arm_joint_targets);
  if (arm_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    arm_group.execute(my_plan);
  }
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(logger, "Stage 5: Rotating to Place position...");
  arm_joint_targets = {1.0, 0.2, 0.2, -0.4}; // Rotate joint1 by ~1 radian
  arm_group.setJointValueTarget(arm_joint_targets);
  if (arm_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    arm_group.execute(my_plan);
  }
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(logger, "Stage 6: Dropping Object...");
  set_gripper(0.019);

  RCLCPP_INFO(logger, "Stage 7: Returning to home posture...");
  arm_joint_targets = {0.0, 0.0, 0.0, 0.0};
  arm_group.setJointValueTarget(arm_joint_targets);
  if (arm_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    arm_group.execute(my_plan);
  }

  RCLCPP_INFO(logger, "--- Pick and Place Operations Successfully Completed! ---");

  rclcpp::shutdown();
  return 0;
}
