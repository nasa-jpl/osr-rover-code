import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    roboclaw_params = os.path.join(
        get_package_share_directory('osr_bringup'),
        'config',
        'roboclaw_params.yaml'
    )
    osr_params = os.path.join(
        get_package_share_directory('osr_bringup'),
        'config',
        'osr_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='osr_control',
            executable='roboclaw_wrapper',
            name='roboclaw_wrapper',
            output='screen',
            emulate_tty=True,
            parameters=[roboclaw_params]
        ),
        Node(
            package='osr_control',
            executable='rover',
            name='rover',
            output='screen',
            emulate_tty=True,
            parameters=[osr_params]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"scale_linear": 0.8},  # scale to apply to drive speed, in m/s: drive_motor_rpm * 2pi / 60 * wheel radius * slowdown_factor
                {"axis_angular": 3},  # which joystick axis to use for driving
                {"scale_angular": 1.75},  # scale to apply to angular speed, in rad/s: scale_linear / min_radius
                {"scale_linear_turbo": 1.78},  # scale to apply to linear speed, in m/s
                {"enable_button": 4},  # which button to press to enable movement
                {"enable_turbo_button": 5}  # -1 to disable turbo
            ],
            remappings=[
                ('/cmd_vel', '/cmd_vel_intuitive')
            ]
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"autorepeat_rate": 5.0},
            ]
        )
    ])
