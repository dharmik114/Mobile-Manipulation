#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

using Trigger = std_srvs::srv::Trigger;

namespace
{
bool move_gripper(
  moveit::planning_interface::MoveGroupInterface & gripper_group,
  const rclcpp::Logger & logger,
  double position)
{
  gripper_group.setStartStateToCurrentState();
  gripper_group.setJointValueTarget(
    std::map<std::string, double>{
      {"gripper_left_joint", position},
      {"gripper_right_joint", position},
    });

  auto result = gripper_group.move();
  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(logger, "Gripper moved to %.3f", position);
    return true;
  }

  RCLCPP_ERROR(logger, "Gripper movement failed with code %d", result.val);
  return false;
}

bool move_arm(
  moveit::planning_interface::MoveGroupInterface & arm_group,
  const rclcpp::Logger & logger,
  const std::vector<double> & target)
{
  arm_group.setStartStateToCurrentState();
  arm_group.setJointValueTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_result = arm_group.plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Arm planning failed with code %d", plan_result.val);
    return false;
  }

  auto execute_result = arm_group.execute(plan);
  if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Arm execution failed with code %d", execute_result.val);
    return false;
  }

  return true;
}

bool run_pick_sequence(
  moveit::planning_interface::MoveGroupInterface & arm_group,
  moveit::planning_interface::MoveGroupInterface & gripper_group,
  const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "Starting pick sequence.");

  if (!move_gripper(gripper_group, logger, 0.019)) {
    return false;
  }

  if (!move_arm(arm_group, logger, {0.0, 0.2, 0.2, -0.4})) {
    return false;
  }

  if (!move_gripper(gripper_group, logger, -0.010)) {
    return false;
  }

  if (!move_arm(arm_group, logger, {0.0, -0.2, 0.4, -0.2})) {
    return false;
  }

  RCLCPP_INFO(logger, "Pick sequence completed.");
  return true;
}
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "pick_object_service",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  moveit::planning_interface::MoveGroupInterface arm_group(node, "manipulator");
  moveit::planning_interface::MoveGroupInterface gripper_group(node, "gripper");

  arm_group.setMaxVelocityScalingFactor(0.5);
  arm_group.setMaxAccelerationScalingFactor(0.5);
  gripper_group.setMaxVelocityScalingFactor(0.3);
  gripper_group.setMaxAccelerationScalingFactor(0.3);

  auto service = node->create_service<Trigger>(
    "/pick_object",
    [&](const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response) {
      (void)request;
      bool ok = run_pick_sequence(arm_group, gripper_group, logger);
      response->success = ok;
      response->message = ok ? "Pick completed" : "Pick failed";
    });

  (void)service;
  RCLCPP_INFO(logger, "/pick_object service ready.");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
