# Setting up the Raspberry Pi (RPi)

In this section, we’ll go over setting up the Raspberry Pi (RPi) and setting up all the code that will run the rover. Our rover uses ROS (Robotic Operating System), which will be installed on your choice of operating system; we will set these up below.

These instructions should work for both the RPi 3 and 4. You are free to use other versions of RPi, ROS, or OS, but setting these up is not covered here and it is not guaranteed that those will work.

## 1 Installing Ubuntu

The first step is to install the Ubuntu Operating System on your Raspberry Pi.

Download Ubuntu 18.04 from [here](https://ubuntu.com/download/raspberry-pi) for your RPi version. Go for the 64 bit version. **Note that as of 7/30/2020, the link for 18.04 64bit for rpi4 is broken, and points to the rpi3 version. You can grab the correct rpi4 version from [this direct link](https://ubuntu.com/download/raspberry-pi/thank-you?version=18.04&versionPatch=.4&architecture=arm64+raspi4)**.

Follow the instructions on the download page for preparing the image for the RPi. Namely:

- Flash Ubuntu onto your microSD card. There are instructions for doing this on [Ubuntu](https://ubuntu.com/tutorials/create-an-ubuntu-image-for-a-raspberry-pi-on-ubuntu#1-overview), [Windows](https://ubuntu.com/tutorials/create-an-ubuntu-image-for-a-raspberry-pi-on-windows), and [Mac](https://ubuntu.com/tutorials/create-an-ubuntu-image-for-a-raspberry-pi-on-macos)
- Attach a monitor and keyboard to the RPi
- Insert the flashed SD card in the RPi
- Power it on
- Login, using "ubuntu" for the username and password

You should now be logged in to your newly minted copy of Ubuntu 18.04 server!

You probably will also want to connect to your newly configured RPi remotely over ssh, rather than having to using a separate monitor every time. Follow these steps.

1. Connect to wifi from the command line
    1. Instructions [here](https://linuxconfig.org/ubuntu-20-04-connect-to-wifi-from-command-line) (basically, you need to edit the `/etc/netplan/50-cloud-init.yaml` file and add your wifi network)
    2. `SSID-NAME-HERE` is your network's name, and `PASSWORD-HERE` is the password for it.
    3. After following these steps, you should see an ip address assigned in the output of `ip a`. It will be an `inet` value like `192.168.1.18`, underneath an interface entry like `wlan0`
2. Enable SSH
    1. Instructions [here]([https://askubuntu.com/a/681768). Namely, run `sudo systemctl enable ssh.socket` from the command line
    3. Now you should be able to login from your dev machine. `ssh ubuntu@192.168.1.18`, or whatever the ip address of your RPi is.
    4. It should prompt you for a password. Once you enter it successfully, you'll be logged! The `enable` step above should configure the ssh server to automatically come up on reboot, so you can just login to the RPi remotely from now on.

<!-- 3. Add your host/development machine's SSH key to the RPi `authorized_keys` file.
    1. Generate an SSH key on your dev machine, if you don't already have one. Use `ssh-keygen` at the command line. (You can choose to give it a password or not - a password probably isn't necessary if you don't plan on putting anything sensitive on your rover)
    2. Copy your public ssh key. `cat ~/.ssh/id_rsa.pub` (or whatever _public_, `.pub`, key file you want). Copy the 
    3.  -->

## 2 Installing ROS

We'll install ROS (Robot Operating System) Melodic on the RPi.

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
<!-- todo: pretty sure these aren't needed anymore. Confirm.
# sensor msgs dependencies
rosinstall_generator sensor_msgs --rosdistro melodic --deps --wet-only --tar >melodic-sensor_msgs-wet.rosinstall
wstool init src melodic-sensor_msgs-wet.rosinstall -->

## 3 Setting up ROS environment and building the rover code

### 3.1 Setup ROS build environment

First we'll create a workspace for the rover code. 

On the Raspberry Pi, open up a terminal `(ctl + alt + t)` and then type the following commands:

```
# Create a catkin workspace directory, which will contain all ROS compilation and 
# source code files, and move into it
mkdir -p ~/osr_ws/src && cd ~/osr_ws

# Source your newly created ROS environment
# EITHER, if you installed ROS melodic
source /opt/ros/melodic/setup.bash
# OR, if you installed ROS kinetic
source /opt/ros/kinetic/setup.bash
```

### 3.2 Clone and build the rover code

In the newly created catkin workspace you just made, clone this repo:
```commandline
cd ~/osr_ws/src
git clone https://github.com/nasa-jpl/osr-rover-code.git

# install the dependencies
rosdep install --from-paths src --ignore-src
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


### 3.3 Add ROS config scripts to .bashrc

The `source...foo.bash` lines above are used to manually configure your ROS environment. We can do this automatically in the future by doing:
```
# use "melodic" or "kinetic" below, as appropriate
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc 
echo "source ~/osr_ws/devel/setup.bash" >> ~/.bashrc
```
This adds the `source` lines to `~/.bashrc`, which runs whenever a new shell is opened on the RPi - by logging in via ssh, for example. So, from now on, when you log into the RPi your new command line environment will have the appropriate configuration for ROS and the rover code.


## 4 Setting up serial communication on the RPi

The RPi will talk to the motor controllers over serial.

Because we are using the serial port for communicating with the roboclaw motor controllers, we have to disable the serial-getty@ttyS0.service service. This service has some level of control over serial devices that we use, so if we leave it on it we'll get weird errors ([source](https://spellfoundry.com/2016/05/29/configuring-gpio-serial-port-raspbian-jessie-including-pi-3-4/)).

Note that this **may** stop us from being able to communicate with the RPi over serial. **Todo**: confirm this and discuss workarounds (e.g. only use ssh)

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

## 5 Testing serial comm with the Roboclaw motors controllers

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