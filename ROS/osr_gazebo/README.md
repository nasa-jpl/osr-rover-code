# ROS packages for JPL Open Source Rover Gazebo Simulation

## Overview
The following ROS packages are included to visualize the rover in rviz and simulate its operations in Gazebo:

- `rviz.launch`: Launches a package for observing the rover in rviz, providing real-time visualization of its movements and sensor data.
- `empty_world.launch`: Deploys the rover within the Gazebo simulation environment, creating a virtual testing ground for rover operations.
- `controller.launch`: Initializes the control system for the rover, setting the stage for user interaction through various input devices.

## Dependencies

### Linux
- Operating System: Ubuntu 20.04.06 LTS
- ROS Distribution: Foxy
- Gazebo Version: 11.14.0

### Required ROS Packages
To fully utilize the capabilities of the rover simulation, the following ROS packages are necessary:

- `rviz`
- `urdf`
- `xacro`
- `gazebo_ros`
- `robot_state_publisher`
- `joint_state_publisher`
- `ros_control`

example:
```bash
sudo apt-get install ros-foxy-controller-manager
sudo apt-get install ros-foxy-robot-state-publisher
sudo apt-get install ros-foxy-joint-state-publisher
sudo apt-get install ros-foxy-gazebo-ros-pkgs
sudo apt-get install ros-foxy-trajectory-msgs
sudo apt-get install ros-foxy-velocity-controllers
```

## Installation

### Create and configure a workspace
Source your ROS installation:
```bash
source /opt/ros/foxy/setup.bash
```
build the osr-gazebo packages:
```bash
cd ~/osr-rover-code/ROS/osr_gazebo
colcon build
source ~/osr-rover-code/ROS/osr_gazebo/install/setup.bash
source devel/setup.bash
```
## Visualisation

### `rover_rviz`

This package includes launch and rviz configuration files for visualising the rover.

To view the rover in rviz and manually control the joints, execute the following command:

```bash
ros2 launch osr_gazebo rviz.launch.py
```
![image](https://github.com/dongjineee/rover_gazebo/assets/150753899/f49548d0-8ecb-4b25-8ce6-bd643bb90b1a)

## Simulation

### `rover_simulation`

This package provides essential launch needed for the visualization of the rover within a simulation environment.

To launch the simulation along with the capability to manually control the joints, use the command:


```bash
ros2 launch osr_gazebo empty_world.launch.py
```
![image](https://github.com/dongjineee/rover_gazebo/assets/150753899/481e0aaf-6336-45e5-b138-49ee7df5e509)

### `rover_control`

To launch the simulation along with the capability to manually control the joints, use the command:
```bash
ros2 launch osr_gazebo controller.launch.py
```
Keyboard controller
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
## Note
- The control does not have specified linear and angular velocities. Therefore, it's necessary to add the maximum and minimum values for `cmd_vel` in the `motor_controller.cpp`.
- The ROS1::noetic version of gazebo simulation exists at https://github.com/dongjineee/rover_gazebo.

