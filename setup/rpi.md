# Setting up the Raspberry Pi (RPi)

In this section, we’ll go over setting up the Raspberry Pi (RPi) and setting up all the code that will run the rover. Our rover uses ROS (Robotic Operating System), which will be installed on your choice of operating system; we will set these up below.

These instructions should work for both the RPi 3 and 4. You are free to use other versions of RPi, ROS, or OS, but setting these up is not covered here and it is not guaranteed that those will work.

## 1 Installing Ubuntu

The first step is to install the Ubuntu Operating System on your Raspberry Pi.

Download Ubuntu 18.04 from [here](https://ubuntu.com/download/raspberry-pi) for your RPi version. For an RPi3 the best default is 32-bit, but if you're on an Rpi4 then you can go for 64-bit.

Preparing the image for the RPi and boot it up:

- Flash Ubuntu onto your microSD card. There are instructions for doing this on [Ubuntu](https://ubuntu.com/tutorials/create-an-ubuntu-image-for-a-raspberry-pi-on-ubuntu#1-overview), [Windows](https://ubuntu.com/tutorials/create-an-ubuntu-image-for-a-raspberry-pi-on-windows), and [Mac](https://ubuntu.com/tutorials/create-an-ubuntu-image-for-a-raspberry-pi-on-macos)
- Attach a monitor and keyboard to the RPi (note an alternative is to use `screen`, see [here](https://elinux.org/RPi_Serial_Connection))
- Insert the flashed SD card in the RPi
- Power it on
- Login, using "ubuntu" for the username and password. You should be prompted to make a new password.

You should now be logged in to your newly minted copy of Ubuntu 18.04!

## 2 Further setup: wifi, desktop GUI (optional), ssh

### 2.1 Connect to wifi from the command line

Get your new device on the internet. Instructions [here](https://linuxconfig.org/ubuntu-20-04-connect-to-wifi-from-command-line). 

1. Basically, you need to edit the `/etc/netplan/50-cloud-init.yaml` file and add your wifi network)
2. `SSID-NAME-HERE` is your network's name, and `PASSWORD-HERE` is the password for it.
3. After following these steps, you should see an ip address assigned in the output of `ip a`. It will be an `inet` value like `192.168.1.18`, underneath an interface entry like `wlan0`

### 2.2 Install a desktop GUI environment (optional)

This is a good option for newbies to the linux world. It's pretty easy to do, though it'll take a while (maybe an hour).

Follow the instructions [here](https://phoenixnap.com/kb/how-to-install-a-gui-on-ubuntu#htoc-update-repositories-and-packages).

1. We recommend using SLiM as the Display Manager, it seems lighter weight than the other options
2. We also recommend using GNOME for the GUI
3. Note that you'll probably need to `sudo tasksel` (instead of without sudo, per the instructions), otherwise you'll get a permissions error.

### 2.3 Enable SSH

You probably will also want to connect to your newly configured RPi remotely over ssh, rather than having to using a separate monitor every time. Instructions [here](https://askubuntu.com/a/681768).

1. Basically, run `sudo systemctl enable ssh.socket` from the command line
3. Now you should be able to login from your dev machine. `ssh ubuntu@192.168.1.18`, using the ip address for your RPi that you found above.
4. It should prompt you for a password. Once you enter it successfully, you'll be logged on! The `enable` step above should configure the ssh server to automatically come up on reboot, so you can just login to the RPi remotely from now on.

## 3 Installing ROS

We'll install ROS (Robot Operating System) Melodic on the RPi.

You'll need to be logged in to the RPi via ssh, or open a terminal in the desktop GUI if you installed it above.

The below steps are based off of [these instructions](http://wiki.ros.org/melodic/Installation/Ubuntu). Consult them for more details.

Setup
```
# Setup your computer to accept software from packages.ros.org
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# set up keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

Installation
```
# make sure your Debian package index is up-to-date
sudo apt update

# desktop-Full Install: (Recommended) : ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators and 2D/3D perception
sudo apt install ros-melodic-desktop-full
```

Dependencies for building packages
```
# install 'em
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

# initialize rosdep
sudo rosdep init
rosdep update
```

## 4 Setting up ROS environment and building the rover code

### 4.1 Setup ROS build environment

First we'll create a workspace for the rover code. 

```
# Create a catkin workspace directory, which will contain all ROS compilation and 
# source code files, and move into it
mkdir -p ~/osr_ws/src && cd ~/osr_ws

# Source your newly created ROS environment
source /opt/ros/$ROS_DISTRO/setup.bash
```

### 4.2 Clone and build the rover code
For this section, you'll be working with the version control software `git`. Now's a good time to [read up](https://try.github.io/) on how that works if you're new to it and make a GitHub account!
In the newly created catkin workspace you just made, clone this repo:
```commandline
sudo apt-get install git
cd ~/osr_ws/src
git clone https://github.com/nasa-jpl/osr-rover-code.git

# install the dependencies
rosdep install --from-paths src --ignore-src
cd ..
catkin_make

# add the generated files to the path so ROS can find them
source devel/setup.bash
```

The rover has some customizable settings that will overwrite the default values. Whether you have any changes compared to the defaults or not, you have to manually create these files:
```
cd ~/osr_ws/src/osr-rover-code/ROS/osr_bringup/config
touch physical_properties_mod.yaml roboclaw_params_mod.yaml
```
To change any values from the default, modify these files instead so they don't get tracked by git. The files follow the same structure as the default.


### 4.3 Add ROS config scripts to .bashrc

The `source...foo.bash` lines above are used to manually configure your ROS environment. We can do this automatically in the future by doing:
```
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc 
echo "source ~/osr_ws/devel/setup.bash" >> ~/.bashrc
```
This adds the `source` lines to `~/.bashrc`, which runs whenever a new shell is opened on the RPi - by logging in via ssh, for example. So, from now on, when you log into the RPi your new command line environment will have the appropriate configuration for ROS and the rover code.


## 5 Setting up serial communication on the RPi

The RPi will talk to the motor controllers over serial.

Because we are using the serial port for communicating with the roboclaw motor controllers, we have to disable the serial-getty@ttyS0.service service. This service has some level of control over serial devices that we use, so if we leave it on it we'll get weird errors ([source](https://spellfoundry.com/2016/05/29/configuring-gpio-serial-port-raspbian-jessie-including-pi-3-4/)).

Note that the following **may** step may stop you from being able to communicate with the RPi over serial.

```
sudo systemctl stop serial-getty@ttyS0.service
sudo systemctl disable serial-getty@ttyS0.service
```


Now we'll need to copy over a udev rules file, which is used to configure needed device files in `/dev`; namely, `ttyS0 and ttyAMA0`. Here's a [good primer](http://reactivated.net/writing_udev_rules.html) on udev. 

```
# copy udev file from the repo to your system
cd ~/osr_ws/src/osr-rover-code/config
sudo cp serial_udev_ubuntu1804.rules /etc/udev/rules.d/10-local.rules

# reload the udev rules so that the devices files are set up correctly.
sudo udevadm control --reload-rules && sudo udevadm trigger
```

This configuration should persist across RPi reboots.

Finally, add the user to the `tty` group:
```
sudo adduser ubuntu tty
```
(or whatever your username is, if not ubuntu)

You'll need to log out of your ssh session and log back in for this to take effect. Or you can restart Ubuntu.


## 6 Testing serial comm with the Roboclaw motors controllers

Run the roboclawtest.py script with all of the motor addresses:
```
cd ~/osr_ws/src/osr-rover-code/scripts
python roboclawtest.py 128
python roboclawtest.py 129
python roboclawtest.py 130
python roboclawtest.py 131
python roboclawtest.py 132
```
Each of these should output something like, within a very short execution time:
```
(1, 'USB Roboclaw 2x7a v4.1.34\n')
(1, 853, 130)
```

If the script seems to hang, or returns only zeros inside the parantheses, then you have a problem communicating with the given roboclaw for that address. Some troubleshooting steps in this cases:

- Make sure you followed the instructions in the "Setting up serial communication" section above, and the serial devices are configured correctly on the RPi.
- Also make sure you went through the calibration instructions from the [main repo](https://github.com/nasa-jpl/open-source-rover/blob/master/Electrical/Calibration.pdf) and set the proper address, serial comm baud rate, and "Enable Multi-Unit Mode" option for every roboclaw controller (if multi-unit mode isn't enabled on every controller, there will be serial bus contention.). If you update anything on a controller, you'll need to fully power cycle it by turning the rover off.
- If you're still having trouble after the above steps, try unplugging every motor controller except for one, and debug exclusively with that one.
