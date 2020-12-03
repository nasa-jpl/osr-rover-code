#!/bin/bash

source osr_paths.sh
launch_dir=$OSR_CODE_DIR/ROS/osr_bringup/launch

bash -c ". /home/$USER/osr_ws/devel/setup.sh"
bash -c ". /home/$USER/osr_ws/devel/setup.bash"
bash -c ". /opt/ros/kinetic/setup.sh"
bash -c ". /opt/ros/kinetic/setup.bash"

# execute the custom mod launch file if it's available
if [ -e "$launch_dir/osr_mod.launch" ]; then
    bash -i -c "roslaunch osr_bringup osr_mod.launch"
# otherwise go with the default
else
    bash -i -c "roslaunch osr_bringup osr.launch"
fi