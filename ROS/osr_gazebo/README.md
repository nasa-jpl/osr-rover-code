# ROS packages for JPL Open Source Rover Gazebo Simulation

> [!NOTE]
> This package isn't compiled with colcon by default. If you want to use this, remove the [COLCON_IGNORE file](./COLCON_IGNORE)

## Overview
The following ROS packages are included to visualize the rover in rviz and simulate its operations in Gazebo:

- `rviz.launch`: Launches a package for observing the rover in rviz, providing real-time visualization of its movements and sensor data.
- `empty_world.launch`: Deploys the rover within the Gazebo simulation environment, creating a virtual testing ground for rover operations.

## Dependencies

The main OSR stack targets **ROS 2 Jazzy**. This simulation package is an exception: it still depends on **Gazebo Classic** (`gazebo_ros` / `gazebo_ros2_control`), which is not supported on Jazzy (Jazzy pairs with Gazebo Harmonic). Until this package is migrated to `ros_gz`, use it on **ROS 2 Humble** with Gazebo Classic.

### Linux (simulation)
- **Operating System**: Ubuntu 22.04 LTS (Jammy)
- **ROS Distribution**: Humble
- **Gazebo**: Classic 11.x

> [!IMPORTANT]
> Do not expect `osr_gazebo` to build or run on Jazzy without migrating away from Gazebo Classic. RViz-only visualization may still work on Jazzy if you only need the URDF/meshes.

## ROS Package Installation
On a Humble machine, install the dependencies:

```bash
sudo apt install python3-colcon-common-extensions
sudo apt-get install ros-humble-rviz2
sudo apt-get install ros-humble-controller-manager
sudo apt-get install ros-humble-robot-state-publisher
sudo apt-get install ros-humble-joint-state-publisher
sudo apt-get install ros-humble-joint-state-publisher-gui
sudo apt-get install ros-humble-gazebo-ros-pkgs
sudo apt-get install ros-humble-trajectory-msgs
sudo apt-get install ros-humble-velocity-controllers
sudo apt-get install ros-humble-joint-trajectory-controller
sudo apt-get install ros-humble-gazebo-ros2-control-demos
```

## Installation

### Create and configure a workspace
Source your ROS installation:
```bash
source /opt/ros/humble/setup.bash
```
Build the osr-gazebo package:
```bash
cd ~/osr-rover-code/ROS/osr_gazebo
colcon build
source ~/osr-rover-code/ROS/osr_gazebo/install/setup.bash
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

Keyboard controller
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
## Note
- The control does not have specified linear and angular velocities. Therefore, it's necessary to add the maximum and minimum values for `cmd_vel` in the controller source (`osr_controller.cpp`).
- A ROS 1 (Noetic) Gazebo simulation exists at https://github.com/dongjineee/rover_gazebo.
- Migrating this package to Gazebo Harmonic / `ros_gz` so it matches the Jazzy rover stack is a known follow-up.

## The method to convert from Onshape to URDF

- The object file for the rover is available within Onshape.
- This object file can be disassembled into its individual components, such as rocker bogie1,2,3, and box, etc..
- However, directly using it as a URDF after converting it to an STL will result in significant CPU and GPU usage in RViz or Gazebo due to the file size issue. 
-  Therefore, the process of reducing the file size of the STL using MeshLab was carried out. 
- The package provided at https://github.com/gstavrinos/calc-inertia was then used to define the inertial properties. 
