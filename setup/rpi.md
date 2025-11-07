# Setting up the Raspberry Pi (RPi)

In this section, we’ll go over setting up the Raspberry Pi (RPi) and setting up all the code that will run the rover. Our rover uses ROS (Robotic Operating System); we will set these up below.

These instructions should work for RPi 3, 4, and 5. You are free to use other versions of RPi, ROS, or OS, but setting these up is not covered here and it is not guaranteed that those will work.

## Installing an Operating System

The first step is to install Ubuntu on your RPi. Select the latest Long Term Support (LTS) version for best results. Other OS'es like Raspberry Pi OS might also work but might require deviation from the instructions, such as building ROS 2 from source. Always check the latest instructions from [the official installation guide](https://docs.ros.org/en/iron/How-To-Guides/Installing-on-Raspberry-Pi.html). We recommend using the [Raspberry Pi Imager](https://www.raspberrypi.com/documentation/computers/getting-started.html#using-raspberry-pi-imager) to load the OS onto an SD card. If done right, you won't need an external monitor, mouse, or keyboard to set up your RPi!

Make sure you set the following settings while configuring the image to be flashed onto the SD card (ctrl/cmd+shift+x):

* Select `Enable SSH` to you can connect to the rover over your local WiFi connection rather than having to attach a screen, keyboard, and mouse to the RPi each time. We recommend using a password.
* `Set username and password`. You will need these later to log into the RPi over SSH.
* `Configure wifi` so the RPi knows how to connect to your network when it boots up.
* `Set locale settings` to your time zone

When done flashing, insert the SD card into the Raspberry Pi. When the Raspberry Pi is powered (make sure to use a proper power supply or connect it to the rover), it should connect to the WiFi connection and be visible from your router's 'devices' page. If possible, we recommend assigning a static IP address to your RPi so you can reach it using that IP address. Otherwise you should also be able to reach it using its hostname (which you selected in the Imager process), potentially with `.local` appended to the end.

From another computer, try connecting to the Raspberry Pi using SSH. You should be able to do so from a terminal in Windows, Linux, or MacOS using the following syntax: `ssh user@machine`, where the user is the username you set during the SD card flashing process earlier. For example:

```bash
ssh osr_user@raspberrypi.local
# or, if you've set a static IP address or know the IP address:
ssh alfred@192.168.1.17
```

It should prompt for your password and then form a connection to a terminal window on the RPi!

> [!TIP]
> Your computer can remember the password for you: replace `ssh` with `ssh-copy-id` in the command above once. Once you've entered your password you should be able to log in without a password!

## Installing ROS

We'll install ROS2 (Robot Operating System) on the RPi. If you're new to ROS, we recommend learning it as it is a crucial part in the code base.

You'll need to be logged in to the RPi via ssh, or open a terminal in the desktop GUI if you're connected via a monitor and mouse/keyboard.

Follow the [instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) for installing ROS2. This takes a while. Follow the instructions in the prompt. You're done when you've completed section "Setup Environment". See below for which version to install. If there are any errors during installation, post them in the `#troubleshooting` channel on Slack to ask for help, if you are not able to resolve them yourself. Just ignoring them will likely cause issues in the future.

> [!NOTE]
> Depending on which Operating System (OS) you installed, you might need to install a [different version of ROS 2](https://www.ros.org/reps/rep-2000.html). Ubuntu and ROS 2 updates frequently and you should check which version of ROS 2 to install depending on what OS you installed! In what follows, we assume ROS 2 `Jazzy`. The OSR code should be independent of which version of ROS 2 you use.

You can choose to either install the 'full version' (`sudo apt install ros-jazzy-desktop`) which comes with graphical packages like RViz and QT or install just the barebones version (`sudo apt install ros-jazzy-ros-base`). The latter allows you to install packages in the full version whenever you need them and so we recommend following this approach. You can install ROS2 and related graphical packages on a different computer on the same network and you will be able to receive all messages out of the box, as long as you're using the same ROS 2 version. If you've installed Ubuntu Server (headless), install the bare bones version since you won't be able to run RViz on the Raspberry Pi anyways!

Check that `ROS 2` is installed correctly. Running `ros2 topic list` from the command line should not say 'Command not found' and `echo $ROS_DISTRO` should show `jazzy` or whichever ROS 2 version you chose to install.

## Setting up ROS environment and building the rover code

### Setup ROS build environment

First we'll create a ROS workspace for the rover code. On the terminal where you've SSH'd into the Raspberry Pi or from the Raspberry Pi's terminal directly (when using the GUI, keyboard, and mouse), run the following commands:

```bash
# Create a colcon workspace directory, which will contain all ROS compilation and
# source code files, and navigate into it
mkdir -p ~/osr_ws/src && cd ~/osr_ws

# Source your newly created ROS environment. If you get "No such file or directory", either you have not installed ROS2 properly, or the environment variables aren't set correctly. Ask for help on Slack on the troubleshooting channel.
source /opt/ros/${ROS_DISTRO}/setup.bash
```

### Clone and build the rover code

For this section, you'll be working with the version control software `git`. Now's a good time to [read up](https://try.github.io/) on how that works if you're new to it and make a GitHub account!
In the newly created colcon workspace you just made, clone (download) this repo:

```bash
sudo apt install git
cd ~/osr_ws/src
git clone https://github.com/nasa-jpl/osr-rover-code.git
```

Now we will install the dependencies using rosdep. Python packages will be installed in a [virtual environment](https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.html#installing-via-a-virtual-environment).

```bash
sudo apt install python3-rosdep python3-pip
cd ..
python3 -m venv venv --system-site-packages
source ./venv/bin/activate
touch ./venv/COLCON_IGNORE
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y
python3 -m pip install adafruit-circuitpython-servokit ina260 RPi.GPIO smbus
# build the ROS packages
python3 -m colcon build --symlink-install
```

It should run successfully. If it doesn't, please ask on Slack or [submit an issue](https://github.com/nasa-jpl/osr-rover-code/issues/new) if you believe there's a problem with the instructions.

Now let's add the generated files to the path so ROS can find them:

```bash
source install/setup.bash
```

The rover has some customizable settings that will overwrite the default values.
Whether you have any changes compared to the defaults or not, you have to manually create these files:

```bash
cd ~/osr_ws/src/osr-rover-code/ROS/osr_bringup/config
touch osr_params_mod.yaml roboclaw_params_mod.yaml
```

In the [rover bringup instructions](rover_bringup.md) we will edit these files to make any changes.

### Add ROS config scripts to .bashrc

The `source ....bash` lines you typed out earlier are used to manually configure your ROS environment. We also want our python virtual environment to be activated. We can do this automatically in the future by doing:

```bash
echo "source ~/osr_ws/venv/bin/activate" >> ~/.bashrc
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo "source ~/osr_ws/install/setup.bash" >> ~/.bashrc
```

This adds the `source` lines to `~/.bashrc`, which runs whenever a new shell is opened on the RPi - by logging in via ssh, for example. So, from now on, when you log into the RPi your new command line environment will have the appropriate configuration for ROS and the rover code.

## Setting up serial communication on the RPi

The RPi talks to the motor controllers over serial bus.

### Enabling Serial and I2C

```bash
sudo apt-get install raspi-config
sudo raspi-config
```

Then use the menu to enable **I2C** and **Serial** under `Interface Options`. When you enable `Serial Port`, it will ask:

* Would you like a login shell to be accessible over serial? --> Select 'No' using the arrow keys and 'tab' and 'enter' keys on your keyboard
* Would you like the serial port hardware to be enabled? --> Yes

If it asks you to reboot, answer 'yes'. Then connect to the Raspberry Pi again once it has rebooted.

More on raspi-config [here](https://www.raspberrypi.com/documentation/computers/configuration.html).

Next, we'll add udev rules to add a symbolic link (symlink) to the serial and i2c devices and configure their permissions:

```bash
sudo cp ~/osr_ws/src/osr-rover-code/config/* /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```
### If using Pi 5, disable the bluetooth serial 
If you are using a Raspberry Pi 5, add the following lines to `/boot/firmware/config.txt`:
```
 enable_uart=1
 dtoverlay=disable-bt
 dtoverlay=uart0
```
This will connect /dev/ttyS0 (and /dev/serial0) to the debug UART, and more importantly, connect /dev/ttyAMA0 (and /dev/serial1) to the hardware UART on GPIO 14/15 that the roboclaws are using.

### Add user to system groups

Finally, add the user to the `tty` `dialout`, and `input` groups on the raspberry pi:

```bash
sudo adduser $USER tty
sudo adduser $USER dialout
sudo adduser $USER input
```

You might have to create the dialout group if it doesn't already exist with `groupadd dialout`.

> [!NOTE]
> You'll need to log out of your ssh session and log back in for this to take effect. Or you can reboot with `sudo reboot`.

Log back in and in a terminal, verify that the serial devices are present:

```bash
ls -l /dev/serial*
```

Should at least show a line that contains `/dev/serial0 -> ttyS0`. This is the main serial device used to send information to the Roboclaws over UART / GPIO pins. If you see `/dev/serial1 -> ttyAMA0`, that is a less powerful software-defined serial device typically used for bluetooth. This varies between Raspberry Pi versions.

## Testing serial comm with the Roboclaw motors controllers

Run the [roboclawtest.py](../scripts/roboclawtest.py) script with all of the motor addresses:

```
cd ~/osr_ws/src/osr-rover-code/scripts
python3 roboclawtest.py 128
python3 roboclawtest.py 129
python3 roboclawtest.py 130
```

Each of these should output something like the following, within a very short execution time:

```
Connected to /dev/serial0.
Address: 130
ReadVersion: (1, 'USB Roboclaw 2x7a v4.2.8\n')
ReadEncM1: (1, 0, 128)
Address 130 - ReadMainBatteryVoltage: (1, 170)
```

The version number may be a later version. If all three work, congratulations! You're close to running the rover. Move onto [rover bringup](rover_bringup.md).

If the script seems to hang, or returns only zeros inside the parantheses (0,0), then you have a problem communicating with the given roboclaw for that address. Some troubleshooting steps in this cases:

- Make sure you followed the instructions in the [#Setting up serial communication] section above, and the serial devices are configured correctly on the RPi.
- Also make sure you went through the calibration instructions from the [main repo](https://github.com/nasa-jpl/open-source-rover/blob/master/Electrical/Calibration.pdf) and set the proper address, serial comm baud rate, and "Enable Multi-Unit Mode" option for every roboclaw controller (if multi-unit mode isn't enabled on every controller, there will be serial bus contention.). If you update anything on a controller, you'll need to fully power cycle it by turning the rover off.
- If you're still having trouble after the above steps, try unplugging every motor controller except for one, and debug exclusively with that one. Reboot the Raspberry Pi if you haven't already.
- If that still doesn't work, please ask on the troubleshooting channel on our Slack group. Include as much relevant information as possible so we can help you find the issue as fast as possible.
