#!/bin/bash
# exit on error, and output executed commands to stdout
set -ex

source osr_paths.sh
launch_dir=$OSR_CODE_DIR/ROS/osr_bringup/launch

bash -c ". /opt/ros/foxy/setup.bash"
bash -c ". /home/$USER/osr_ws/install/setup.sh"

# execute the custom mod launch file if it's available
if [ -e "$launch_dir/osr_mod_launch.py" ]; then
    echo "Launching osr_mod_launch.py"
    bash -i -c "ros2 launch osr_bringup osr_mod_launch.py"
# otherwise go with the default
else
    echo "Launching osr.launch"
    bash -i -c "ros2 launch osr_bringup osr_launch.py"
fi
