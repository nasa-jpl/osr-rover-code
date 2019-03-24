# JPL's Open Source Rover ROS Control Code
This branch is an in-development version of the Open Source Rover Code, running in ROS Kinetic. This will be updated with more in depth instructions on how the code works as well as instructions as its' development progresses. 

## Installing
First you must install ROS onto your Raspberry Pi. Instructions for this can be found at:
  * [http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi)


On your Raspberry Pi run the following commands in a terminal

`mkdir -p ~/osr_ws/src`
`cd ~/osr_ws/src`
`git clone https://github.com/nasa-jpl/osr-rover-code/tree/osr-ROS`


## Running

There is a roslaunch file that will start all of the nodes in the ROS system, to run it:

`roslaunch osr_bringup osr.launch`



