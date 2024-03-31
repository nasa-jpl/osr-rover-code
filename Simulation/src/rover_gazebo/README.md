# ROS packages for JPL Open Source Rover Gazebo Simulation

## Introduction
This package suite is designed to bring the JPL Open Source Rover to life in a simulated environment using Gazebo. Inspired by [NASA JPL's Open Source Rover project](https://github.com/nasa-jpl/open-source-rover), these packages allow for visualization and control in a Gazebo simulation.

## Overview
The following ROS packages are included to visualize the rover in rviz and simulate its operations in Gazebo:

- `rover.launch`: Launches a package for observing the rover in rviz, providing real-time visualization of its movements and sensor data.
- `rover_gazebo.launch`: Deploys the rover within the Gazebo simulation environment, creating a virtual testing ground for rover operations.
- `controller.launch`: Initializes the control system for the rover, setting the stage for user interaction through various input devices.
- `rover_teleop_keyboard.launch`: A package that allows the user to maneuver the rover using keyboard inputs, ensuring precise and responsive control.
- `rover_teleop_xbox.launch`: Enables control of the rover via an Xbox controller, offering an intuitive and ergonomic option for navigation and operation.

## Dependencies

### Linux
- Operating System: Ubuntu 20.04.06 LTS
- ROS Distribution: Noetic
- Gazebo Version: 11.14.0

### Required ROS Packages
To fully utilize the capabilities of the rover simulation, the following ROS packages are necessary:

- `rviz`
- `urdf`
- `xacro`
- `gazebo_ros`
- `robot_state_publisher`
- `joint_state_publisher`
- `diagnostic_updater`
- `ros_control`

example:
```bash
sudo apt-get install ros-noetic-ros-controllers
sudo apt-get install ros-noetic-diagnostic-updater
sudo apt-get install ros-noetic-robot-state-publisher
sudo apt-get install ros-noetic-joint-state-publisher
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

## Installation

### Create and configure a workspace
Source your ROS installation:
```bash
source /opt/ros/noetic/setup.bash
```
Create a catkin workspace:
```bash
mkdir -p ~/rover_ws/src
cd ~/rover_ws/src
catkin_init_workspace
cd ..
catkin_make
```
Clone and build the packages:
```bash
cd ~/rover_ws/src
git clone https://github.com/dongjineee/rover_gazebo.git
cd ..
catkin_make
source devel/setup.bash
```
## Visualisation

### `rover_rviz`

This package includes launch and rviz configuration files for visualising the rover.

To view the rover in rviz and manually control the joints, execute the following command:

```bash
roslaunch rover rover.launch
```
![image](https://github.com/dongjineee/rover_gazebo/assets/150753899/f49548d0-8ecb-4b25-8ce6-bd643bb90b1a)

## Simulation

### `rover_simulation`

This package provides essential launch needed for the visualization of the rover within a simulation environment.

To launch the simulation along with the capability to manually control the joints, use the command:


```bash
roslaunch rover rover_gazebo.launch
```
![image](https://github.com/dongjineee/rover_gazebo/assets/150753899/481e0aaf-6336-45e5-b138-49ee7df5e509)

### `rover_control`

To launch the simulation along with the capability to manually control the joints, use the command:
```bash
roslaunch rover controller.launch
```
Keyboard controller
```bash
roslaunch rover rover_teleop_keyboard.launch
```
To execute this package, first clone the repository using the following command:

```bash
git clone https://github.com/methylDragon/teleop_twist_keyboard_cpp.git
```
xbox controller
```bash
roslaunch rover rover_teleop_xbox.launch
```

### `moon_world`

places the rover in a Moon terrain model sourced from https://github.com/MobileRobots/amr-ros-config/tree/master/gazebo
```bash
roslaunch rover moon_world.launch
```

![image](https://github.com/dongjineee/rover_gazebo/assets/150753899/900263f7-dad4-45c1-9c6b-41af9d975a6f)

## Note
- When you run the Gazebo and control launch files, you may encounter a message stating `No p gain specified for pid`. This message can safely be ignored. The reason is that the `*_wheel_joint_*` entities are intended to function as servos.
- The control does not have specified linear and angular velocities. Therefore, it's necessary to add the maximum and minimum values for `cmd_vel` in the `motor_controller.cpp`.
- There are two types of odometry topics available: `gt_odometry`, which is based on ground truth data, and another that derives from the position of the `wheel_joint`.

## Issue
- The joint type for rocker_bogie 1&2 has been set to 'revolute' to provide degrees of freedom. However, doing so results in the actual tf relationship not being connected between the box and rocker_bogie2. It's necessary to give freedom to the bogie while not affecting the tf relationship.
