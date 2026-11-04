"""Drive the rover in an empty Gazebo Harmonic world (ROS 2 Jazzy).

Starts Gazebo, publishes the robot description, spawns the rover, bridges
Gazebo's clock into ROS, and starts the ros2_control controllers that drive the
wheels and steering (via the gz_ros2_control plugin in the robot description).

On top of that it runs the same rover node as the real robot, so a Twist on
/cmd_vel or /cmd_vel_intuitive goes through the real kinematics:

  Twist -> rover (osr_control) -> /cmd_drive + /cmd_corner
        -> gazebo_command_adapter -> wheel_controller + servo_controller

Drive it with the keyboard (in a second terminal):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_intuitive
or with a gamepad connected to this computer:
  ros2 launch osr_gazebo harmonic_world.launch.py joystick:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
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

    # The real rover's control node: turns a Twist into wheel speeds and corner
    # angles. Odometry is off because the sim has no encoder feedback wired in.
    # Dimensions come from the robot's osr_params.yaml, except the wheel radius,
    # which must match the simulated wheel (82 mm, the collision cylinder in
    # osr.urdf.xacro) so commanded speeds come out right whatever wheels the
    # physical rover uses.
    rover = Node(
        package='osr_control',
        executable='rover',
        name='rover',
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('osr_bringup'), 'config', 'osr_params.yaml']),
            {
                'rover_dimensions.wheel_radius': 0.082,
                'enable_odometry': False,
                'publish_transform': False,
                'use_sim_time': True,
            },
        ],
    )

    # Translates the rover's motor commands into ros2_control controller commands
    # (fixes the sign conventions that differ between the hardware and the model)
    adapter = Node(
        package='osr_gazebo',
        executable='gazebo_command_adapter.py',
        name='gazebo_command_adapter',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Optional gamepad teleop. Mapping mirrors osr_bringup/launch/osr_launch.py
    # (DualSense: hold L1, left stick drive / spin, right stick steer, R1 turbo)
    joystick = LaunchConfiguration('joystick')
    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
        condition=IfCondition(joystick),
        parameters=[{'autorepeat_rate': 5.0, 'device_id': 0, 'use_sim_time': True}],
    )
    teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        condition=IfCondition(joystick),
        parameters=[{
            'scale_linear.x': 0.4,
            'axis_linear.x': 1,
            'axis_angular.yaw': 3,
            'axis_angular.pitch': 0,
            'scale_angular.yaw': 0.9,
            'scale_angular.pitch': 1.0,
            'scale_angular_turbo.yaw': 3.95,
            'scale_linear_turbo.x': 1.78,
            'enable_button': 4,
            'enable_turbo_button': 5,
            'use_sim_time': True,
        }],
        remappings=[('cmd_vel', 'cmd_vel_intuitive')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('joystick', default_value='false',
                              description='Start joy + teleop_twist_joy for a gamepad'),
        gazebo, robot_state_publisher, spawn, clock_bridge, controllers,
        rover, adapter, joy, teleop,
    ])
