import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    roboclaw_params = os.path.join(
        get_package_share_directory('osr_bringup'),
        'roboclaw_params',
        'roboclaw_params.yaml'
    )
    osr_params = os.path.join(
        get_package_share_directory('osr_bringup'),
        'osr_params',
        'osr_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='osr_control',
            executable='roboclaw_wrapper.py',
            name='roboclaw_wrapper',
            output='screen',
            parameters=[roboclaw_params]
        ),
        Node(
            package='osr_control',
            executable='rover.py',
            name='rover',
            output='screen',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])