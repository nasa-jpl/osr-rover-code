# OSR startup service

The files in this folder allow you to start any rover related processes automatically when the device starts up.
Don't follow these instructions if your rover is still under heavy development.
This is useful for example to swap out batteries in the field while the rover is driving or restart when a bug occurs.

## Setting up the systemd service

1. If you haven't already, edit your ~/.bashrc files to contain `source /opt/ros/$ROS_DISTRO/setup.bash` and 
`source /home/yourusername/catkin_ws/devel/setup.bash` (modifying for your catkin workspace) using `sudo nano ~/.bashrc`.
2. Make sure your current working directory is this folder, e.g. `cd ~/catkin_ws/src/osr-rover-code/init_scripts`
3. Copy the service to the `systemd` folder: `sudo cp osr.service /etc/systemd/system/`
4. Copy the executable to the `/usr/bin` folder: `sudo cp LaunchOSR.sh /usr/bin/`
5. Set the correct permissions on the service: `sudo chmod 644 /etc/systemd/system/osr.service`

## Testing the service

Before we activate the service to run on startup, let's test it out first:

`sudo service osr start`

Will launch ROS through `roslaunch osr_bringup osr.launch`.

If you see an error, something went wrong. 
Even when you don't get any output, check the contents of the service status:

`sudo service osr status`

If you see an error, [open an issue](https://github.com/nasa-jpl/osr-rover-code/issues/new) with details of your system
and the contents of the service status call. Note that this call shows output from previous calls as well, so watch the
time stamps.

If all went well, `rosnode list` should show a list of active ROS nodes that are running. If you see 
`ERROR: Unable to communicate with master!`, something went wrong.

## Activating the service

Activate the service so it runs on boot:

`sudo systemctl enable osr.service`

Test that it works by turning the robot off and on again, or using `sudo reboot`.

If you want to deactivate it:

`sudo systemctl disable osr.service`
