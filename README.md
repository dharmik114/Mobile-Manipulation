# Mobile Manipulation Robot

This project develops a mobile robot that can autonomously retrieve objects on command. A TurtleBot3 Waffle Pi carries an OpenManipulator-X arm. The robot navigates to a known object location, detects the target cube with an RGB-D camera, picks it up, delivers it to a drop location and returns to its start position.

The project is developed in simulation with GazeboSim, starting with navigation and manipulation as separate subsystems and progressing toward a complete, automatic fetch-and-deliver workflow.

## Current Features

- Gazebo simulation of the combined mobile manipulator with LiDAR, IMU and RGB-D camera, connected to ROS 2 through `ros_gz_bridge`.
- Autonomous navigation with Nav2 on a pre built map, with static and dynamic obstacle avoidance.
- Odometry and IMU fusion with an EKF.
- MoveIt 2 configuration for the arm and gripper and `ros2_control` trajectory controllers.
- Pick-and-hold and move-and-place routines: the arm moves to a given position, closes or opens the gripper, and returns to a fixed hold pose.
- Planned: object detection, behavior tree task sequencing and a task manager interface.

## Working of Navigation and Manipulation



https://github.com/user-attachments/assets/7df87195-c4df-49a9-b048-1a337d34db3c



## Repository Structure

| Package | Description |
|---|---|
| `mobman_description` | Robot model (URDF/xacro), Gazebo worlds, sensor plugins, `ros_gz_bridge` and EKF configuration, simulation launch files |
| `mobman_navigation` | Nav2 configuration (parameters, costmaps), saved map, navigation launch file and `nav_to_pose.py` |
| `mobman_moveit_config` | MoveIt 2 configuration: planning groups, kinematics, joint limits, `ros2_control` controllers, `move_group` launch file |
| `manipulation_operation` | C++ nodes for arm motion: `pick_and_hold_node`, `move_and_place_node`, `pick_and_place_node` |

## Development Timeline

- [x] Set up the TurtleBot3 Waffle Pi simulation.
- [x] Implement autonomous navigation using Nav2.
- [x] Test obstacle avoidance.
- [x] Integrate the OpenManipulator-X manipulator.
- [x] Configure MoveIt 2 for motion planning.
- [x] Implement object pick-and-hold and move-and-place with the arm.
- [ ] Integrate the RGB-D camera for cube detection and localization.
- [ ] Sequence navigation and manipulation with a behavior tree.
- [ ] Build a task manager interface with cube selection, cancel and emergency stop.
- [ ] Implement object retrieval commands, such as "Bring me the red cube."

## Installation

Clone the repository:

```bash
git clone https://github.com/dharmik114/Mobile-Manipulation.git
cd Mobile-Manipulation
```

Install dependencies and build the workspace:

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Running the Project

Run each command in a separate terminal (source `install/setup.bash` in each).

```bash
# 1. Gazebo simulation and robot
ros2 launch mobman_description display.launch.py use_sim_time:=true
```
```bash
# 2. Navigation (Nav2)
ros2 launch mobman_navigation navigation.launch.py use_sim_time:=true
```
```bash
# 3. Motion planning (MoveIt 2)
ros2 launch mobman_moveit_config move_group.launch.py use_sim_time:=true
```

Set the initial pose in RViz first, then send a navigation goal with the RViz "2D Goal Pose" tool.

Run the arm routines (the arm moves as soon as the node starts):

```bash
# Pick the object and move to the hold pose
ros2 run manipulation_operation pick_and_hold_node --ros-args -p use_sim_time:=true
```
```bash
# Move to the place position, release the object and return to the hold pose
ros2 run manipulation_operation move_and_place_node --ros-args -p use_sim_time:=true
```

## Future Goals

The final goal is a mobile manipulation system that combines autonomous navigation, object perception and robotic arm control to retrieve and deliver objects with minimal human intervention: one button press (red, green or blue cube) starts a fully automatic run, with cancel and emergency-stop support. Deployment on the real robot is planned as future work.
