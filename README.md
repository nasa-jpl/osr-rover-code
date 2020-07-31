# JPL Open Source Rover Code

This repository contains the code that runs on the Raspberry Pi and Arduino to control the 
[JPL open source rover (osr)](https://github.com/nasa-jpl/open-source-rover).
This includes the Arduino code that controls the LED matrix.
The rover runs on ROS1 and its code currently is agnostic of the specific distribution 
(tested on Kinetic, Melodic, Noetic). All Python code is currently Python2.

Status: actively developed. Please refer to the issues tab in GitHub for an overview of ongoing work.

## 1. Flashing the Arduino Code

In this section we will be flashing the code that runs on the arduino to control the LED matrix in the head. The following steps should be performed on your laptop or development machine (not the raspberry pi)

1. Install the Arduino IDE used for loading code onto the arduino: https://www.arduino.cc/en/Main/Software
2. Download the Arduino code:
    1. Navigate to https://github.com/nasa-jpl/osr-rover-code and click the green ’Clone or download’
button. Choose ’Download ZIP’.
    2. Unzip/extract and open the downloaded zip file. Then, select the Arduino folder and create a
new zip file of just that Arduino folder. Name it OsrScreen.zip
3. Load the sketch onto the Arduino
    1. Unplug the Arduino shield JST cable so the Arduino isn’t powered by the control board
    2. Connect the Arduino to your development machine with USB cable
    3. Open Arduino IDE
    4. Select Sketch - Include Library - Add .Zip Library
    5. Select the OsrScreen.zip folder created previously
    6. Click the Upload button in the Sketch Window
4. To load the example in the Arduino IDE: File -> Examples -> OsrScreen -> OsrScreen

## 2. Setting up the Raspberry Pi

todo: is bluetooth pairing a thing in these instructions?
In this section, we’ll go over setting up the Raspberry Pi (rpi) and setting up all the code that will run the rover. Our rover uses ROS (Robotic Operating System), which will be installed on your choice of operating system; we will set these up below. We will also set up the bluetooth pairing from the android device.

These instructions should work for both the rpi 3 and 4. You are free to use other versions of Raspberry Pi, ROS, or OS, but setting these up is not covered here and it is not guaranteed that those will work.

### 2.1 Installing the Operating System

The first step is to install the Ubuntu (Recommended), or Raspbian OS on your Raspberry Pi.

todo: should probably add ssh setup here

#### 2.1.1 Ubuntu 18.04

You may be able to use a different version of Ubuntu, but YMMV.

Download Ubuntu 18.04 from https://ubuntu.com/download/raspberry-pi for your rpi version. Go for the 64 bit version. **Note that as of 7/30/2020, the link for 18.04 64bit for rpi4 is broken, and points to the rpi3 version. You can grab the correct rpi4 version from https://ubuntu.com/download/raspberry-pi/thank-you?version=18.04&versionPatch=.4&architecture=arm64+raspi4**

Follow the instructions on the download page for preparing the image for the rpi. Namely:

- Flash Ubuntu onto your microSD card. The page has instruction for doing this on [Ubuntu](https://ubuntu.com/tutorials/create-an-ubuntu-image-for-a-raspberry-pi-on-ubuntu#1-overview), Windows, and Mac
- Attach a monitor and keyboard to the rpi
- Insert the flashed SD card in the rpi
- Power it
- Login, using "ubuntu" for the username and password

You should now be logged in to your newly minted copy of Ubuntu 18.04 server!


#### 2.1.2 Raspbian

To install Raspbian, we recommend following the "Getting started with Raspberry Pi" tutorial below. 

**NOTE: These instructions were tested and verified using Raspbian Stretch. Newer versions of Raspbian (such as Buster) may cause complications with these instructions. We therefore recommend installing Raspbian Stretch instead of the latest/default version of Raspbian, which the tutorial below will recommend. Do not flash NOOBS onto your card; instead download the Raspbian Streth image below and skip the NOOBS step in the tutorial.** 

In addition, we have created a pre-built image with both Raspbian Stretch and ROS already installed. You can download this image as a last resort if you can’t get Raspbian and ROS installed and working on your own, but please note that you will miss out on some the learning opportunities that come with debugging a software installation by doing so. 

- Tutorial for installing the latest version of Raspbian on a new Raspberry Pi: https://projects.raspberrypi.org/en/projects/raspberry-pi-getting-started
- Direct downloadable Raspbian Stretch image: https://downloads.raspberrypi.org/raspbian/images/raspbian-2019-04-09/2019-04-08-raspbian-stretch.zip
- Custom JPL image with both Raspbian Stretch and ROS pre-built: https://drive.google.com/drive/folders/1RbYWDRthpcktqkPEHb7qVqDiy27EiAc1

todo: step number below
**NOTE: If you install the Custom JPL image with Raspbian Stretch and ROS pre-built, skip to step 2.4.**

Once you finish the above tutorial and install Raspbian Stretch, you should be able to boot your Raspberry Pi and see a desktop!

## 2.2 Installing ROS

Continue from page 4 in the Software Steps pdf


### Raspberry Pi

2. In the newly created catkin workspace you just made, clone this repo:
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

### Arduino

Please refer to the instructions in 
[Software Steps](https://github.com/nasa-jpl/open-source-rover/blob/master/Software/Software%20Steps.pdf)

## Bringup rover

In a sourced terminal (`source ~/osr_ws/devel/setup.bash`), run

```commandline
roslaunch osr_bringup osr.launch
```
to run the rover.
Any errors or warnings will be displayed there in case something went wrong. If you're using the Xbox wireless controller,
command the rover by holding the left back button (LB) down and moving the joysticks.

## Internals & Structure

Please refer to README files associated with each folder for insight in how components work and what they do. 
This is also the place to look when you have modifications on your rover that require the code or parameters to be
changed.

* The [ROS overview](ROS/README.md) gives an overview of the setup related to ROS and links to specific implementations
such as how the drive and corner commands are being calculated
* The [Arduino readme](Arduino/README.md) details the code that runs on the Arduino, used to control the LED screen.
