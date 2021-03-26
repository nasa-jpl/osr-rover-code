#!/bin/bash
bash -c ". /opt/ros/foxy/setup.sh"
bash -c ". /opt/ros/foxy/setup.bash"
bash -c ". /home/$USER/osr_ws/install/setup.sh"
bash -c ". /home/$USER/osr_ws/install/setup.bash"
bash -i -c "ros2 launch osr_bringup osr_launch.py"
