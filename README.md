# JPL Open Source Rover Code

This repository contains the code that runs on the Raspberry Pi (RPi) to control the 
[JPL open source rover (osr)](https://github.com/nasa-jpl/open-source-rover).

The rover runs on ROS2 (tested on Foxy), and uses Python3.

## Internals & structure

Please refer to README files associated with each folder for insight in how components work and what they do. 
This is also the place to look when you have modifications on your rover that require the code or parameters to be
changed.

* The [ROS overview](ROS/README.md) gives an overview of the setup related to ROS and links to specific implementations such as how the drive and corner commands are being calculated

## Simulation

A `Dockerfile` is provided to build and run the ROS2 stack, including the
[Gazebo Classic simulation](ROS/osr_gazebo/README.md), without installing anything on your host
besides Docker.

```bash
docker build -t osr-rover-sim .
```

To run a node or launch file with a graphical window (e.g. Gazebo, RViz), the container needs
access to your display. The easiest way to get that without hand-rolling X11/GPU flags yourself is
[`rocker`](https://github.com/osrf/rocker), which detects your setup (including an NVIDIA GPU) and
wires up the right flags automatically:

```bash
pip install rocker
rocker --nvidia --x11 osr-rover-sim
```

> [!NOTE]
> `--nvidia` must not be the last flag before the image name: it takes an optional value
> (`auto`/`runtime`/`gpus`), so `rocker --x11 --nvidia osr-rover-sim` fails with
> `error: argument --nvidia: invalid choice: 'osr-rover-sim'` because argparse swallows the
> image name as that value.

Inside the container:

```bash
source install/setup.bash
ros2 launch osr_gazebo empty_world.launch.py
```

## Setup

The setup directory contains tutorial files for getting everything setup and configured for running the rover code. You should proceed through them in the following order:

* [Raspberry Pi setup](setup/rpi.md)
* [Rover code bringup](setup/rover_bringup.md)
