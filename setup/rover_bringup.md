# Bringing up the rover code

Note that these instructions assume you followed the steps in [rpi setup](rpi.md).

## 1 Manual rover bringup

In a sourced terminal (`source /opt/ros/$ROS_DISTRO/setup.bash && source ~/osr_ws/devel/setup.bash`), run

```commandline
roslaunch osr_bringup osr.launch
```
to run the rover.

Any errors or warnings will be displayed there in case something went wrong. If you're using the Xbox wireless controller,
command the rover by holding the left back button (LB) down and moving the joysticks.

## 2 Automatic bringup with init script

Starting scripts on boot using ROS can be a little more difficult than starting scripts on boot normally from
the Raspberry Pi because of the default permission settings on the RPi and the fact that that ROS cannot
be ran as the root user. The way that we will starting our rover code automatically on boot is to create
a service that starts our roslaunch script, and then automatically run that service on boot of the robot.
[Further information](https://www.linode.com/docs/quick-answers/linux/start-service-at-boot/) on system service scripts running at boot.

There are two scripts in the ”Software/Init Scripts” folder. The first is the bash file that runs the
roslaunch file, and the other creates a system service to start that bash script. Open up a terminal on the
raspberry Pi and execute the following commands.
```
cd /home/pi/osr/Init\ Scripts
sudo cp LaunchOSR.sh /usr/bin/LaunchOSR.sh
sudo chmod +x /usr/bin/LaunchOSR.sh
sudo cp osr startup.service /etc/systemd/system/osr startup.service
sudo chmod 644 /etc/systemd/system/osr startup.service
```

Your osr startup service is now installed on the Pi and ready to be used. The following are some commands
related to managing this service which you might find useful:

| Description | Command |
| --- | --- |
| Start service | sudo systemctl start osr startup.service |
| Stop service | sudo systemctl stop osr startup.service |
| Enable service (runs on boot of RPi) | sudo systemctl enable osr startup.service |
| Disable service (doesn’t run on boot of RPi) | sudo systemctl disable osr startup.service |
| Check status of service | sudo systemctl status osr startup.service |
| View live service list | sudo journalctl -f |

**Note: We do not recommend enabling the service until you have verified that everything
on your robot runs successfully manually. Once you enable the service, as soon as you power
on the RPi it will try and run everything. This could cause issues if everything has not yet
been fully tested and verified. Additionally, if you are doing development of your own software
for the robot we suggest disabling the service and doing manual launch of the scripts during
testing phases. This will help you more easily debug any issues with your code.**

Once you have fully tested the robot and made sure that everything is running correctly by starting the rover code manually
via `roslaunch osr bringup osr.launch`, enable the startup service on the robot with the command below:
```
sudo systemctl enable osr startup.service
```

At this point, your rover should be fully functional and automatically run whenever you boot it up! Congratulations and happy roving!!
