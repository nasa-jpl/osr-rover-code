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

### Communication Packet Structure

#### Initilization Packet
There is an initilization exchange between the Raspberry Pi and Arduino which is used to let each device know the other one is ready to send/receive data. Until each device has correctly agreed upon them they will both be in an IDLE state. 

The Raspberry Pi will continually send its' initilization command at a 2Hz frequency until it recieves the Arduino's initilization command. The Arduino sits and listens constantly until it hears this command, and then transistions into a RUNNING state, and sends its' initilization command. While in IDLE the RPi listens for this command, and will also transistion to RUNNING upon receiving it. 


|    `Name`     |      `Value`     |  
|---------------|------------------|
| RPi Init      | 0xA              |
| Arduino Init  | 0xCD             |

Once both of them have agreed upon recieving each other's initilization command they will both be in a RUNNING state sending the normal telemetry packet. If either system hits the timeout constant on listening for the telemetry packet they will be transistioned back to the IDLE state, in which initilization must occur again.

#### Preamble
The preamble (or syncword) is used to do bit-level syncronization of the data frames between the two system. It is the first 2 bytes of every single message.

|               |      |      |      |      |  
|---------------|------|------|------|------|
| Binary        | 1010 | 1011 | 1100 | 1101 |             
| Hexadecimal   | A    | B    | C    | D    |

#### CRC Checksum
The CRC-16 Checksum provides verification that the frame was received correctly. It is computed off the data contained inside the frame and is stored at the end of the sequence that can be re-calculated on both sides of the data stream. It is calculated as the uint16_t(SUM (bytes[2,14]), or the SUM of all bytes in the data packet except for the preamble (and itself of course)

#### Data Frames
The data frame is a total of 16 bytes sent, and will have the following structure

| `PREAMBLE` | `CONN` | `BATTERY` | `ERROR`  |   `TEMP`  | `DRIVE_CURRENT` | `STEERING_CURRENT` | `FACE`  | `CHECKSUM` |  
|------------|--------|-----------|----------|-----------|-----------------|--------------------|---------|------------|
| 0xABCD     | uint8_t| uint8_t   | uint8_t  |uint8_t(3) |   uint8_t(3)    |      uint8_t(2)    | uint8_t |  uint8_t(2)|


##### CONN
Connected Status of the remote controller to the Robot. 1 Byte length

| `Value` | `Description`                                           |
|---------|---------------------------------------------------------|
| 0x01    | Remote connected                                        |
| 0x00    | Remote disconnected                                     |
| 0xFF    | Overloaded to transistion the arduino back to IDLE state|

##### BATTERY
Battery value read in by the motor controllers. 1 Byte length. The value ranges from 0x00 to 0x1F, based on battery %

| `Value` | `Battery % ` |
|---------|--------------|
| 0x00    | 0            |
| 0x01    | 20           |
| 0x03    | 40           |
| 0x07    | 60           |
| 0x0F    | 80           |
| 0x1F    | 100          |

##### Status
Error status of each individual motor controller. 1 Byte length. Each motor controller error status is x << i bits, where x indicates an error, is 1 if error and 0 otherwise

| Motor Controler Address | Binary (if error) |
|-------------------------|-------------------|
| 128                     | 0001 0000         | 
| 129                     | 0000 1000         |
| 130                     | 0000 0100         |
| 131                     | 0000 0010         |
| 132                     | 0000 0001         |

The error to send is now built out of the bitwise OR of all these together, such that if motor controllers 131 and 129 had errors the error message would be `0000 1010`

##### TEMP
Temperature reading coming off of each RoboClaw. 3 Bytes length, where each nibble corresponds to a motor controller, with possible nibble values from 0x0 to 0x4, which ranges from temperatures 28-40 C. The TEMP values are binned into every 20% of the total range, which are also shown below

| `NIBBLE NUM`| 0     |  1    |   2   |   3   |   4   |   5   |
|-------------|-------|-------|-------|-------|-------|-------|
| `RoboClaw Num`| N/A | 128   | 129   | 130   | 131   | 132   |

| `Temp value`   | `Hex Value` |
|:--------------:|-------------|
|       T <30.4  | 0x0         |
|30.4 < T < 32.8 | 0x1         |
|32.8 < T < 35.2 | 0x2         |
|35.2 < T < 37.6 | 0x3         |
|37.6 < T        | 0x4         |


##### DRIVE_CURRENT
Driving motors current reading coming off of each RoboClaw. 3 Bytes length, where each nibble corresponds to a motor controller, with possible nibble values from 0x0 to 0x4, which ranges from current 0-1 amps. The current values are binned into every 20% of the total range, which are also shown below

| `NIBBLE NUM`| 0     |  1    |   2   |   3   |   4   |   5   |
|-------------|-------|-------|-------|-------|-------|-------|
| `Motor Number`| 1   | 2     |  3    | 4     | 5     | 6     |

| `Temp value`   | `Hex Value` |
|:--------------:|-------------|
|      A < 0.2   | 0x0         |
|0.2 < A < 0.4   | 0x1         |
|0.4 < A < 0.6   | 0x2         |
|0.6 < A < 0.8   | 0x3         |
|0.8 < A         | 0x4         |



##### STEERING_CURRENT
Steering motors current reading coming off of each RoboClaw. 2 Bytes length, where each nibble corresponds to a motor controller, with possible nibble values from 0x0 to 0x4, which ranges from current 0-1 amps 

| `NIBBLE NUM`| 0     |  1    |   2   |   3   |
|-------------|-------|-------|-------|-------|
| `Motor Number`|  7  | 8     | 9     | 10    |

| `Temp value`   | `Hex Value` |
|:--------------:|-------------|
|       A < 0.2  | 0x0         |
|0.2 < A < 0.4   | 0x1         |
|0.4 < A < 0.6   | 0x2         |
|0.6 < A < 0.8   | 0x3         |
|0.8 < A         | 0x4         |



##### FACE
Command for the face. __TO DO: ADD THIS FUNCTIONALITY IN STILL, FOR NOW FACE IS STATIC__


#### Example 
In order to understand the data telemetry frame it is easiest to just run through an example with data from the robot. The following data is in the format which is passed from the ROSTOPIC /status

| `Parameter` | `Value`               |
|-------------|-----------------------|
| CONN        | 1                     |
| BATTERY     | 167                   |
| STATUS      | [0,1,1,1,0]           |
| TEMP        | [312,314,340,310,300] |
| DRIVE_CURRENT| [8,8,6,5,4,6]        |
| STEER_CURRENT| [9,5,9,6]            |
| FACE         | 1                    |

First each of these values must be converted into its' respective values, and then inserted into the correct spot into the data frame. 

The Battery is full, and will be 0x1F

Motor controllers 129, 130, 131 have errors, which means that the STATUS binary code generated should be: `0000 1110`

Using the above information in the TEMP description we can see the TEMP should bytes should be 0x01, 0x12, 0x10

Using the above information in the TEMP description we can see the DRIVE_CURRENT should bytes should be 0x44, 0x32, 0x23

Using the above information in the TEMP description we can see the STEERING_CURRENT should bytes should be 0x42, 0x43

The face will be 0x01

Now that all the data is put into the right form it is time to compute the CHECKSUM, which is the SUM of all of the data bytes. It is important to also note that this value could overflow the 1 BYTE size, and thus has 2 bytes to describe it, a HIGH BYTE and LOW BYTE. 

Adding all these values together produces the number 337. To put this into the data frame we will need to break it into it's high byte and low byte values. The binary Representation of 337 is `0000 0001 0101 0001`


                                   337           <=>   0000 0001 0101 0001
                                   high byte      =    0000 0001
                                   low byte       =               0101 0001
                                   
 
 Putting this all together gives the following data telemetry packet:
 
 |     |`PREAMBLE_HIGH`|`PREAMBLE_LOW`| `CONN`| `BATTERY` | `STATUS` | `TEMP` | `D_CURRENT` | `S_CURRENT` | `FACE` | `CHK_HIGH` | `CHK_LOW`|
 |-----|---------------|--------------|----------|----------|----------|----------|----------|----------|----------|----------|----------|
 | Byte Num| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 | Byte 8 | Byte 9 | Byte 10 | Byte 11 | Byte 12| Byte 13| Byte 14 | Byte 15|          
 | Decimal | 171     | 205    | 1      | 17     | 14    | 1       | 18     | 17     | 68     | 50     | 35      | 66      | 67     | 1     | 1       | 81     |
 | Hex     | 0xAB    | 0xCD   | 0x01   | 0x1F   | 0x0E  | 0x01    | 0x12   | 0x10   | 0x44   | 0x32   | 0x23    | 0x42    | 0x43   | 0x01 | 0x01      | 0x51   | 
 | Binary  |  10101011 | 11001101 | 00000001 | 00011111 | 00000111 | 00000001 | 000010010 | 00010000 | 01000100 | 00110010 | 00100011 | 01000010 | 01000011 | 00000001 | 00000001 | 001010001 |
 
 
 
