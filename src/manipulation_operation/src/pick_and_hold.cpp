#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

using moveit::planning_interface::MoveGroupInterface;

// ============================ EDIT VALUES HERE ============================
// Pick position (Cartesian, frame: base_footprint, link: end_effector_link)
constexpr double kPickX = 0.246;
constexpr double kPickY = 0.0;
constexpr double kPickZ = 0.010;

// Gripper closed position (same value as your original code)
constexpr double kGripperClosedPosition = -0.009;

// Hold pose in JOINT space (radians): j1=0, j2=0, j3=0, j4=45 deg.
// This equals the Cartesian pose measured with tf2_echo:
//   xyz = (0.157, 0.0, 0.216), rpy = (0, 0.790, 0)
constexpr double kHoldJoint1 = 0.0;
constexpr double kHoldJoint2 = 0.0;
constexpr double kHoldJoint3 = 0.0;
constexpr double kHoldJoint4 = 0.7854;
// ==========================================================================

// Plan + execute whatever target is currently set on `group`.
static bool planAndExecute(MoveGroupInterface &group, const rclcpp::Logger &logger,
                           const std::string &name)
{
  group.setStartStateToCurrentState();

  MoveGroupInterface::Plan plan;
  const auto plan_result = group.plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "[%s] planning failed with error code %d.", name.c_str(), plan_result.val);
    return false;
  }

  const auto execute_result = group.execute(plan);
  if (execute_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "[%s] execution failed with error code %d.", name.c_str(), execute_result.val);
    return false;
  }

  RCLCPP_INFO(logger, "[%s] done.", name.c_str());
  return true;
}

static int run(const rclcpp::Node::SharedPtr &node)
{
  const auto logger = node->get_logger();

  MoveGroupInterface arm(node, "arm");
  arm.setPoseReferenceFrame("base_footprint");
  arm.setEndEffectorLink("end_effector_link");
  arm.setPlanningTime(5.0);

  MoveGroupInterface gripper_group(node, "gripper");

  if (!arm.getCurrentState(5.0))
  {
    RCLCPP_ERROR(
        logger,
        "No current robot state received. Check that /joint_states is published and that "
        "this node has use_sim_time:=true when Gazebo is used.");
    return 1;
  }

  // ---- Step 1: move to the pick position (Cartesian, position only) ----
  arm.setStartStateToCurrentState();
  if (!arm.setPositionTarget(kPickX, kPickY, kPickZ, "end_effector_link"))
  {
    RCLCPP_ERROR(logger, "Could not set the position target for end_effector_link.");
    return 1;
  }
  if (!planAndExecute(arm, logger, "pick position"))
    return 1;
  RCLCPP_INFO(logger, "Arm reached the pick position.");

  // ---- Step 2: close the gripper ----
  gripper_group.setStartStateToCurrentState();
  const std::map<std::string, double> gripper_closed_target{
      {"gripper_left_joint", kGripperClosedPosition},
      {"gripper_right_joint", kGripperClosedPosition},
  };
  if (!gripper_group.setJointValueTarget(gripper_closed_target))
  {
    RCLCPP_ERROR(logger, "Could not set the gripper closed target.");
    return 1;
  }
  if (!planAndExecute(gripper_group, logger, "close gripper"))
    return 1;
  RCLCPP_INFO(logger, "Gripper closed.");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // let the grip settle

  // ---- Step 3: go to the hold pose (joint space) ----
  if (!arm.getCurrentState(5.0))
  {
    RCLCPP_ERROR(
        logger,
        "No current robot state received. Check that /joint_states is published and that "
        "this node has use_sim_time:=true when Gazebo is used.");
    return 1;
  }

  const std::map<std::string, double> hold_joint_target{
      {"joint1", kHoldJoint1},
      {"joint2", kHoldJoint2},
      {"joint3", kHoldJoint3},
      {"joint4", kHoldJoint4},
  };
  arm.setStartStateToCurrentState();
  if (!arm.setJointValueTarget(hold_joint_target))
  {
    RCLCPP_ERROR(logger, "Could not set the hold joint target.");
    return 1;
  }
  if (!planAndExecute(arm, logger, "hold pose"))
    return 1;

  RCLCPP_INFO(logger, "Holding the object at the hold pose. Press Ctrl+C to exit.");
  return 0;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "pick_and_hold_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // MoveGroupInterface needs callbacks to receive /joint_states, /tf, and /clock.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  const int exit_code = run(node);

  // On success keep the node alive so the arm and gripper keep holding.
  if (exit_code == 0)
  {
    while (rclcpp::ok())
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return exit_code;
}