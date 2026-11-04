# ROS packages for JPL Open Source Rover Gazebo Simulation

## Overview
Visualization and simulation for the OSR, in two flavors:

| Simulator | ROS 2 | Ubuntu | Launch file |
|---|---|---|---|
| **Gazebo Harmonic** (`ros_gz`, `gz_ros2_control`) | Jazzy | 24.04 | `harmonic_world.launch.py` |
| Gazebo Classic 11 (`gazebo_ros`, `gazebo_ros2_control`) | Humble | 22.04 | `empty_world.launch.py` |

- `rviz.launch.py`: view the rover URDF in RViz
- `harmonic_world.launch.py` / `empty_world.launch.py`: spawn the rover and drive it with the **same** kinematics as the hardware stack

Both share one URDF (`urdf/osr.urdf.xacro`); the xacro argument `sim:=harmonic|classic` (default `classic`) selects the simulator-specific plugins.

### Control architecture

Simulation does not reimplement rocker-bogie math. Instead:

1. `osr_control/rover` consumes `/cmd_vel` (or `/cmd_vel_intuitive`) and publishes `/cmd_drive` + `/cmd_corner` using shared `osr_control.kinematics` and dimensions from `osr_bringup/config/osr_params.yaml`
2. `gazebo_command_adapter.py` bridges those commands to the `wheel_controller` and `servo_controller` (ros2_control), fixing the sign conventions that differ between the hardware and the model, and ramping wheel speeds at a limited acceleration like the RoboClaws do (`wheel_acceleration`, default 18 rad/s²; 0 disables)

Changing `d1`–`d4` in `osr_params.yaml` therefore affects hardware and simulation the same way. In Harmonic, `wheel_radius` is overridden to 0.082 m to match the simulated wheel (see [Known differences](#known-differences)).

## Gazebo Harmonic (ROS 2 Jazzy)

### Dependencies

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-controllers ros-jazzy-ros2controlcli \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2 \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-teleop-twist-joy ros-jazzy-joy
```

### Build

From the workspace root (so `osr_control` / `osr_interfaces` / `osr_bringup` are available):

```bash
source /opt/ros/jazzy/setup.bash
cd ~/osr_ws
colcon build --packages-select osr_interfaces osr_control osr_bringup osr_gazebo
source install/setup.bash
```

### Run

```bash
ros2 launch osr_gazebo harmonic_world.launch.py
```

This starts Gazebo with an empty world, spawns the rover, bridges `/clock`, starts the controllers (`joint_state_broadcaster`, `wheel_controller`, `servo_controller`), the `rover` node and the command adapter.

Drive with the keyboard in a second terminal (`i` forward, `,` back, `u`/`o` arcs, `k` stop):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_intuitive
```

Or with a gamepad connected to the simulation computer (hold the enable button, default L1 / button 4):

```bash
ros2 launch osr_gazebo harmonic_world.launch.py joystick:=true
```

Check the controllers with `ros2 control list_controllers`.

### Troubleshooting

- **Gazebo window freezes, or the terminal hangs at "Requesting list of world names"**: Gazebo's transport discovery is being blocked, often by a firewall (e.g. `ufw`) on the network interface. Keep Gazebo on the local machine with `export GZ_IP=127.0.0.1` (add it to `~/.bashrc` to make it permanent).

### Known differences

- **Collision shapes**: Harmonic uses simple shapes for collisions (cylinders for the six drive wheels, a box for the body, none for the other links) instead of the detailed meshes, which kept the simulation at roughly half real time. Visuals still use the meshes. On flat ground only the wheels make contact; rough terrain would need boxes for the rocker-bogie links.
- **Wheel size**: the wheel mesh is 164 mm × 101 mm, which appears to be the previously recommended Traxxas tires. The current parts list uses goBILDA Wasteland Wheels (144 mm × 52 mm), and `osr_params.yaml` uses `wheel_radius: 0.075`. The Harmonic launch overrides `wheel_radius` to 0.082 so commanded speeds match the simulated wheel.
- No IMU sensor yet (the Classic IMU plugin is not ported).

## Gazebo Classic (ROS 2 Humble)

### Linux (simulation)
- **Operating System**: Ubuntu 22.04 LTS (Jammy)
- **ROS Distribution**: Humble
- **Gazebo**: Classic 11.x
- **Also required**: `osr_control`, `osr_interfaces`, `osr_bringup` (built in the same workspace)

### ROS Package Installation

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

### Installation

Build from the ROS workspace root (so `osr_control` / `osr_interfaces` / `osr_bringup` are available):

```bash
source /opt/ros/humble/setup.bash
cd ~/osr_ws   # workspace that contains this repo's ROS/ packages
colcon build --packages-select osr_interfaces osr_control osr_bringup osr_gazebo
source install/setup.bash
```

### Visualisation

```bash
ros2 launch osr_gazebo rviz.launch.py
```

### Simulation

```bash
ros2 launch osr_gazebo empty_world.launch.py
```

Drive with:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

> [!NOTE]
> Rotate-in-place on the hardware stack uses `Twist.angular.y` (joystick pitch). `teleop_twist_keyboard` only commands `angular.z`, so in-place rotation needs a publisher that sets `angular.y` with zero `linear.x`, or the intuitive/joy setup from `osr_bringup`.

## Follow-ups
- Bridge Gazebo `/joint_states` into `/drive_state` and `/corner_state` so `rover` odometry can run in sim
- Port the IMU sensor to Harmonic
- Update the wheel mesh to the goBILDA Wasteland Wheel

## The method to convert from Onshape to URDF

- The object file for the rover is available within Onshape.
- This object file can be disassembled into its individual components, such as rocker bogie1,2,3, and box, etc..
- However, directly using it as a URDF after converting it to an STL will result in significant CPU and GPU usage in RViz or Gazebo due to the file size issue.
- Therefore, the process of reducing the file size of the STL using MeshLab was carried out.
- The package provided at https://github.com/gstavrinos/calc-inertia was then used to define the inertial properties.
