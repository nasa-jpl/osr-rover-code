# JPL's Open Source Rover ROS Control Code
This branch is an in-development version of the Open Source Rover Code. It contains code to run the rover in ROS kinetic, as well as the Arduino Library to use in conjunction with the LED matrix and arduino Sheild pcb. This will be updated with more in depth instructions on how the code works as well as instructions as its' development progresses. 

## ROS

### Installing
First you must install ROS onto your Raspberry Pi. Instructions for this can be found at:
  * [http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi)


On your Raspberry Pi run the following commands in a terminal

`mkdir -p ~/osr_ws/src`

`cd ~/osr_ws/src`

`git clone https://github.com/nasa-jpl/osr-rover-code/tree/osr-ROS/ROS`


<img src="img/OSR_ROS_Diagram.png" width="100%">

### Catkin Packages
There are 4 catkin packages that this repo houses. Each of these packages performs a specific purpose in the ROS structure, which will be covered below
 
  * osr_msgs - 
  * osr_bringup - Roslaunch process to start all nodes necessary to run the robot
  * led_screen - Contains code to communicate to the Arduino Uno to run the LED screen

#### osr
Contains the scripts necessary to run the physical robot.
  * xbox_controller.py - Remaps Xbox controller input to commands for the robot and led screen
  * robot_node.py -      Runs ROS node based on the robot class
  * robot.py -           Does computation on how to distribute motor speeds to move rover correctly
  * rc_mc_node.py -      Runs ROS node based on the Controller class
  * rc_mc.py -           Communicates commands/data to/from the RoboClaw motor controllers

#### osr_msgs
Contains the message definitions/types used in the topics for the rover

__NEED TO UPDATE ALL MESSAGE TYPES TO INT32 INSTEAD OF INT64__

##### Commands
The Commands message is used to pass commands to the motors, in the following format:

|    `Name`     |      `Type`      |   `Element data range` |
|---------------|------------------|------------------------|
| drive_motor   | int32[6]         | [-100,100]             |
| corner_motor  | int32[4]         | [0,2000]               |

  * The drive_motor data element is a 6 element list of values from -100 to 100, where 100 corresponds to the speed magnitude and (+) and (-) are the direction. 

  * The corner_motor data element is a 4 element list where the values will be pulled automatically from the calibration process on the RoboClaws. These values are the physical encoder tick value targeted for the position command

##### Encoder
The Encoder message is to convey information from the RoboClaws about their current position so the robot can divide up driving wheel speed appropriately 

|    `Name`     |      `Type`      |   `Element data range` |
|---------------|------------------|------------------------|
| abs_enc       | int32[4]         | [0,2000]               |
| abs_enc_ange  | int32[4]         | [-45,45]               |

  * The abs_enc data element are the physical encoder ticks that the corner absolute encoders are reading
  * The abs_enc_angles are the angles calculated based on individual calibrations of the corner systems

##### Joystick
The Joystick message is to give generic inputs to the robot node to do driving calculations on.
__NEED TO UPDATE THE JOYSTICK MESSAGE DATA VALUES TO MATCH THE DESCRIPTIONS__

|    `Name`     |      `Type`      |   `Element data range` |
|---------------|------------------|------------------------|
| velocity      | int32            | [-100,100]             |
| steering      | int32            | [-100,100]             |

  * Velocity corresponds to the drive percent of the rover, positive is forward negative is backwards
  * Steering corresponds to the turning of the rover, positive turns to the right, negative turns to the left

##### Status
The Status message takes data from the RoboClaw motor controllers and populates it into the ROS system, to be displayed onto the LED screen

|    `Name`     |      `Type`      | 
|---------------|------------------|
| battery       | int32            |
| error_status  | int32[5]         |
| temp          | int32[5]         |
| current       | int32[5]         |

  * battery data element is the voltage level being read into the RoboClaws. The value is 10 x [Battery Voltage(V)]
  * error_status is the status of an error being present on each RoboClaw. 1 if Error, 0 otherwise
  * temp is the temperature read on each RoboClaw. The value is 10 x (Temperature[C])
  * current is the current drawn on each motor. The value is 10 x (Current(A))

##### Screen
__TO DO: ADD THIS MESSAGE TYPE__
The Screen message takes input from the controller for controller the LED face

|    `Name`     |      `Type`      |   `Element data range` |
|---------------|------------------|------------------------|
| face          | int32            | [0,5]                  |
| connected     | int32            | 0 or 1                 |

#### osr_bringup 
The osr_bringup package contains the Roslaunch file necessary to start all the ROS nodes, as well as the parameter set of operating parameters for the robot.

##### ROS Params

|    `Name`     |      `Type`      | Value                     |
|:--------------|:-----------------|:--------------------------|
| /baud_rate    | int64            | 115200                    |
| /device       | string           | "/dev/serial0"            |
| /addresses    | string           | "128,129,130,131,132"     |
| /enc_min      | string           | "0,0,0,0"                 |
| /enc_max      | string           | "0,0,0,0"                 |
| /qpps         | string           | "0,0,0,0"                 |
| /accel        | string           | "0,0,0,0"                 |
| /battery_low  | int32            | 11                        |
| /battery_high | int32            | 18                        |
| /mech_dist    | string           | "7.254,10.5,10.5,10.073"  |


### Running

There is a roslaunch file that will start all of the nodes in the ROS system, to run it:

`roslaunch osr_bringup osr.launch`

## Arduino

### Installing
On your development machine (not Raspberry Pi) first you need to install Arduino IDE
  * [https://www.arduino.cc/en/Main/Software](https://www.arduino.cc/en/Main/Software)

  * Clone the arduino library
  
  `git clone https://github.com/nasa-jpl/osr-rover-code/tree/osr-ROS/Arduino OsrScreen`

  * Select the OsrScreen folder and create a .ZIP folder out of it
  * Open arduiono IDE
    * Sketch -> Include Library -> Add .ZIP Library
      * Select the OsrScreen.ZIP folder

  * To load the example in Arduino IDE
    * File -> Examples -> Osr-Screen -> Osr-Screen


