"""Spawn the rover in an empty Gazebo Harmonic world (ROS 2 Jazzy).

Starts Gazebo, publishes the robot description, spawns the rover, bridges
Gazebo's clock into ROS, and starts the ros2_control controllers that drive the
wheels and steering (via the gz_ros2_control plugin in the robot description).
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution([FindPackageShare('osr_gazebo'), 'urdf', 'osr.urdf.xacro'])
    # Expand the xacro template into plain URDF, selecting the Harmonic variant
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' sim:=harmonic']), value_type=str)

    # Gazebo Harmonic with its built-in empty world; -r starts the physics running
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Publishes the URDF on /robot_description and the positions of the rover's parts (TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # Reads /robot_description and drops the rover into the world, just above the ground
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'rover', '-z', '0.05'],
    )

    # Gazebo keeps its own simulated clock; bridge it to ROS /clock so nodes using
    # use_sim_time follow simulation time
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    # Start the controllers once the rover (and its controller_manager) exists.
    # spawner waits for the controller_manager, so no extra sequencing is needed.
    #  - joint_state_broadcaster: publishes joint positions on /joint_states
    #  - wheel_controller: wheel speeds from /wheel_controller/commands
    #  - servo_controller: steering angles from /servo_controller/joint_trajectory
    controllers = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', 'wheel_controller', 'servo_controller'],
    )

    return LaunchDescription([gazebo, robot_state_publisher, spawn, clock_bridge, controllers])
