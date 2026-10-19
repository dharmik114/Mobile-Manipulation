# Mobile Manipulation Robot

This project aims to develop a mobile robot capable of autonomously retrieving objects on command. Using a TurtleBot3 Waffle equipped with an X-series manipulator, the robot will navigate to a known object location, detect the target using an Intel RealSense camera, pick it up, and deliver it to a specified location.

The project is being developed in simulation, starting with autonomous navigation and progressing toward a complete pick-and-place system.

## Current Features

- Autonomous navigation using Nav2.
- Static and dynamic obstacle avoidance.
- Gazebo simulation of the mobile robot.
- Planned integration of object detection and robotic manipulation.

## Navigation Demonstration

The navigation subsystem has been implemented and tested in simulation.

## Development Timeline

- [x] Set up the TurtleBot3 Waffle simulation.
- [x] Implement autonomous navigation using Nav2.
- [x] Test obstacle avoidance.
- [ ] Integrate the X-series manipulator.
- [ ] Configure MoveIt 2 for motion planning.
- [ ] Integrate the Intel RealSense camera for object detection and localization.
- [ ] Implement object grasping and placement.
- [ ] Integrate navigation and manipulation into a complete workflow.
- [ ] Implement object retrieval commands, such as "Bring me the red cube."

## Installation

Clone the repository:

```bash
git clone https://github.com/dharmik114/Mobile-Manipulation/tree/main
cd <YOUR_REPOSITORY_NAME>
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

Launch the simulation and navigation using the launch files provided in the repository.

```bash
ros2 launch mobman_description display.launch.py 
```
```bash
# Replace this path with the absolute path to your own map file.
ros2 launch mobman_navigation navigation.launch.py map:=<Absolute_Path_to_>/src/mobman_navigation/map/my_map.yaml 
```

## Future Goals

The final goal is to create a mobile manipulation system that combines autonomous navigation, object perception, and robotic arm control to retrieve and deliver objects with minimal human intervention.
