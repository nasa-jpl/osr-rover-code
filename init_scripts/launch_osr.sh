#!/bin/bash
# exit on error, and output executed commands to stdout
set -ex

source osr_paths.sh
launch_dir=$OSR_CODE_DIR/ROS/osr_bringup/launch

bash -c ". /home/$USER/osr_ws/install/setup.sh"
bash -c ". /home/$USER/osr_ws/devel/setup.bash"
bash -c ". /opt/ros/melodic/setup.sh"
bash -c ". /opt/ros/melodic/setup.bash"

# execute the custom mod launch file if it's available
if [ -e "$launch_dir/osr_mod.launch" ]; then
    echo "Launching osr_mod.launch"
    bash -i -c "roslaunch osr_bringup osr_mod.launch"
# otherwise go with the default
else
    echo "Launching osr.launch"
    bash -i -c "roslaunch osr_bringup osr.launch"
fi
