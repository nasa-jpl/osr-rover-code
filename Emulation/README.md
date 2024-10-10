# Rover Emulation

This project provides a Docker-based emulation environment for a rover control system using ROS 2 Foxy.

We currently have command and control emulation 

Possible things we can add:

Odometer emulation
Sensor emulation

## Setup Instructions

### 1. Clone the Repository

```bash
git clone https://github.com/untrobotics/rover_emulation
cd rover_emulation
```

### 2. Build the Docker Image

Replace `{IMAGE_NAME}` with your preferred image name (use lowercase).

```bash
docker build -t {IMAGE_NAME} .
```

### 3. Run the Docker Container

Replace `{CONTAINER_NAME}` with your preferred container name.

```bash
docker run -it --name {CONTAINER_NAME} {IMAGE_NAME}
```

### 4. Launch the Rover

In the initial terminal:

```bash
ros2 launch osr_bringup osr_launch.py
```

### 5. Open Additional Terminals

To open new terminals connected to the running container:

```bash
docker exec -it {CONTAINER_NAME} /bin/bash
```

Note: You may need to source the ROS 2 setup files in new terminals:

```bash
source /opt/ros/foxy/setup.bash
source /root/osr_ws/install/setup.bash
```

## Available Commands

Here are some basic commands to control the rover. Run these in a separate terminal from the one running the launch file.

### Move Forward

To move the rover forward at half speed:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Move Backward

To move the rover backward at half speed:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Turn Left

To turn the rover left at a specific angular velocity (in radians/second):

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

This command will make the rover turn left at approximately 28.6 degrees per second (0.5 radians/second).

### Turn Right

To turn the rover right at a specific angular velocity (in radians/second):

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}"
```

This command will make the rover turn right at approximately 28.6 degrees per second (-0.5 radians/second).

### Stop the Rover

To stop all movement:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Troubleshooting

If you encounter issues with command recognition in new terminals, ensure you've sourced the ROS 2 setup files as mentioned in the setup instructions.

For any other issues or questions, please open an issue in the GitHub repository.
