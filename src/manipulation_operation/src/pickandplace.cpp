#include <memory>
#include <map>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "pick_and_place_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const auto logger = node->get_logger();

  // MoveGroupInterface needs callbacks to receive /joint_states, /tf, and /clock.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface arm(node, "arm");
  arm.setPoseReferenceFrame("base_footprint");
  arm.setEndEffectorLink("end_effector_link");
  arm.setPlanningTime(5.0);

  int exit_code = 0;
  if (!arm.getCurrentState(5.0)) {
    RCLCPP_ERROR(
      logger,
      "No current robot state received. Check that /joint_states is published and that "
      "this node has use_sim_time:=true when Gazebo is used.");
    exit_code = 1;
  } else if (!arm.setPositionTarget(0.050, 0.0, 0.450, "end_effector_link")) {
    RCLCPP_ERROR(logger, "Could not set the position target for end_effector_link.");
    exit_code = 1;
  } else {
    arm.setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = arm.plan(plan);

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Planning failed with error code %d.", plan_result.val);
      exit_code = 1;
    } else {
      const auto execute_result = arm.execute(plan);
      if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Execution failed with error code %d.", execute_result.val);
        exit_code = 1;
      } else {
        RCLCPP_INFO(logger, "Arm reached the position target.");

        moveit::planning_interface::MoveGroupInterface gripper_group(node, "gripper");
        gripper_group.setStartStateToCurrentState();

        // Keep the target just inside the -0.010 m mechanical lower limit.
        constexpr double kGripperClosedPosition = 0.009;
        const std::map<std::string, double> gripper_closed_target{
          {"gripper_left_joint", kGripperClosedPosition},
          {"gripper_right_joint", kGripperClosedPosition},
        };
        if (!gripper_group.setJointValueTarget(gripper_closed_target)) {
          RCLCPP_ERROR(logger, "Could not set the gripper closed target.");
          exit_code = 1;
        } else {
          moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
          const auto gripper_plan_result = gripper_group.plan(gripper_plan);

          if (gripper_plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(logger, "Gripper planning failed with error code %d.", gripper_plan_result.val);
            exit_code = 1;
          } else {
            const auto gripper_execute_result = gripper_group.execute(gripper_plan);
            if (gripper_execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
              RCLCPP_ERROR(
                logger, "Gripper execution failed with error code %d.", gripper_execute_result.val);
              exit_code = 1;
            } else {
              RCLCPP_INFO(logger, "Gripper closed.");
            }
          }
        }
      }
    }
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return exit_code;
}
