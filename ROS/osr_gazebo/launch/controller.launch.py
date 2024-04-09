import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro

def generate_launch_description():

    controller_spawn = Node(
        package='osr_gazebo',
        executable='osr_controller',
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'cartpole'],
        output='screen'
    )

    # joint_state_controller
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_state_broadcaster'],
        output='screen'
    )

    # wheel_velocity_controller
    load_joint_trajectory_controller_velocity_mr = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'rover_motor_mr_controller'],
        output='screen'
    )

    load_joint_trajectory_controller_velocity_ml = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'rover_motor_ml_controller'],
        output='screen'
    )

    load_joint_trajectory_controller_velocity_fr = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'rover_motor_fr_controller'],
        output='screen'
    )

    load_joint_trajectory_controller_velocity_fl = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'rover_motor_fl_controller'],
        output='screen'
    )

    load_joint_trajectory_controller_velocity_rr = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'rover_motor_rr_controller'],
        output='screen'
    )

    load_joint_trajectory_controller_velocity_rl = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'rover_motor_rl_controller'],
        output='screen'
    )

    # servo_controller
    rover_servo_fr_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'rover_servo_fr_controller'],
        output='screen'
    )

    rover_servo_fl_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'rover_servo_fl_controller'],
        output='screen'
    )

    rover_servo_rr_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'rover_servo_rr_controller'],
        output='screen'
    )

    rover_servo_rl_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'rover_servo_rl_controller'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    load_joint_state_controller,
                    load_joint_trajectory_controller_velocity_mr,
                    load_joint_trajectory_controller_velocity_ml,
                    load_joint_trajectory_controller_velocity_fr,
                    load_joint_trajectory_controller_velocity_fl,
                    load_joint_trajectory_controller_velocity_rr,
                    load_joint_trajectory_controller_velocity_rl,
                ],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[
                    rover_servo_fr_controller,
                    rover_servo_fl_controller,
                    rover_servo_rr_controller,
                    rover_servo_rl_controller,
                ],
            )
        ),
        controller_spawn,
        spawn_entity,
    ])
