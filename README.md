# JPL Open Source Rover Code

This repository contains the code that runs on the Raspberry Pi (RPi) to control the 
[JPL open source rover (osr)](https://github.com/nasa-jpl/open-source-rover).

The rover runs on ROS2 (tested on Foxy), and uses Python3.

## Internals & structure

Please refer to README files associated with each folder for insight in how components work and what they do. 
This is also the place to look when you have modifications on your rover that require the code or parameters to be
changed.

* The [ROS overview](ROS/README.md) gives an overview of the setup related to ROS and links to specific implementations
such as how the drive and corner commands are being calculated

## Setup

The setup directory contains tutorial files for getting everything setup and configured for running the rover code. You should proceed through them in the following order:

* [Raspberry Pi setup](setup/rpi.md)
* [Rover code bringup](setup/rover_bringup.md)
