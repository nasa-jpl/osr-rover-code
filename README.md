# JPL Open Source Rover Code

This repository contains the code that runs on the Raspberry Pi and Arduino to control the 
[JPL open source rover (osr)](https://github.com/nasa-jpl/open-source-rover).
This includes the Arduino code that controls the LED matrix.
The rover runs on ROS1 and its code currently is agnostic of the specific distribution 
(tested on Kinetic, Melodic, Noetic). All Python code is currently Python2.

Status: actively developed. Please refer to the issues tab in GitHub for an overview of ongoing work.

## Installation

### Raspberry Pi

1. Follow the instructions to set up your Raspberry Pi and install ROS in 
[Software Steps](https://github.com/nasa-jpl/open-source-rover/blob/master/Software/Software%20Steps.pdf)
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
