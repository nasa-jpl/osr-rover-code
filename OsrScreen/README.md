# Arduino

This folder is not a part of the ROS package structure. It contains the code that controls the Arduino.

## Communication Packet Structure

This section describes the internals of communication between the Raspberry Pi and the Arduino. You don't need
to understand these if you just want to get your rover running.

### Initilization Packet
There is an initilization exchange between the Raspberry Pi and Arduino which is used to let each device know the other 
one is ready to send/receive data. Until each device has correctly agreed upon them they will both be in an IDLE state. 

The Raspberry Pi will continually send its initilization command at a 2Hz frequency until it recieves the Arduino's 
initilization command. The Arduino sits and listens constantly until it hears this command, and then transistions into 
a RUNNING state, and sends its' initilization command. While in IDLE the RPi listens for this command, and will also 
transistion to RUNNING upon receiving it. 

|    `Name`     |      `Value`     |  
|---------------|------------------|
| RPi Init      | 0xA              |
| Arduino Init  | 0xCD             |

Once both of them have agreed upon recieving each other's initilization command they will both be in a RUNNING state 
sending the normal telemetry packet. If either system hits the timeout constant on listening for the telemetry packet 
they will be transistioned back to the IDLE state, in which initilization must occur again.

### Preamble
The preamble (or syncword) is used to do bit-level syncronization of the data frames between the two system. 
It is the first 2 bytes of every single message.

|               |      |      |      |      |  
|---------------|------|------|------|------|
| Binary        | 1010 | 1011 | 1100 | 1101 |             
| Hexadecimal   | A    | B    | C    | D    |

### CRC Checksum
The CRC-16 Checksum provides verification that the frame was received correctly. 
It is computed off the data contained inside the frame and is stored at the end of the sequence that can be 
re-calculated on both sides of the data stream. It is calculated as the uint16_t(SUM (bytes[2,14]), 
or the SUM of all bytes in the data packet except for the preamble (and itself of course)

### Data Frames
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

#### Status
Error status of each individual motor controller. 1 Byte length. Each motor controller error status is x << i bits, 
where x indicates an error, is 1 if error and 0 otherwise

| Motor Controler Address | Binary (if error) |
|-------------------------|-------------------|
| 128                     | 0001 0000         | 
| 129                     | 0000 1000         |
| 130                     | 0000 0100         |
| 131                     | 0000 0010         |
| 132                     | 0000 0001         |

The error to send is now built out of the bitwise OR of all these together, such that if motor controllers 131 and 129 
had errors the error message would be `0000 1010`

#### TEMP
Temperature reading coming off of each RoboClaw. 3 Bytes length, where each nibble corresponds to a motor controller, 
with possible nibble values from 0x0 to 0x4, which ranges from temperatures 28-40 C. The TEMP values are binned into 
every 20% of the total range, which are also shown below

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


#### DRIVE_CURRENT
Driving motors current reading coming off of each RoboClaw. 3 Bytes length, where each nibble corresponds to a motor 
controller, with possible nibble values from 0x0 to 0x4, which ranges from current 0-1 amps. 
The current values are binned into every 20% of the total range, which are also shown below

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
Steering motors current reading coming off of each RoboClaw. 2 Bytes length, where each nibble corresponds to a motor 
controller, with possible nibble values from 0x0 to 0x4, which ranges from current 0-1 amps 

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

#### FACE
Command for the face. __TO DO: ADD THIS FUNCTIONALITY IN STILL, FOR NOW FACE IS STATIC__


### Example 
In order to understand the data telemetry frame it is easiest to just run through an example with data from the robot. 
The following data is in the format which is passed from the ROS topic `/status`

| `Parameter` | `Value`               |
|-------------|-----------------------|
| CONN        | 1                     |
| BATTERY     | 167                   |
| STATUS      | [0,1,1,1,0]           |
| TEMP        | [312,314,340,310,300] |
| DRIVE_CURRENT| [8,8,6,5,4,6]        |
| STEER_CURRENT| [9,5,9,6]            |
| FACE         | 1                    |

First each of these values must be converted into its' respective values, and then inserted into the correct spot into 
the data frame. 

The Battery is full, and will be 0x1F

Motor controllers 129, 130, 131 have errors, which means that the STATUS binary code generated should be: `0000 1110`

Using the above information in the TEMP description we can see the TEMP should bytes should be 0x01, 0x12, 0x10

Using the above information in the TEMP description we can see the DRIVE_CURRENT should bytes should be 0x44, 0x32, 0x23

Using the above information in the TEMP description we can see the STEERING_CURRENT should bytes should be 0x42, 0x43

The face will be 0x01

Now that all the data is put into the right form it is time to compute the CHECKSUM, which is the SUM of all of the 
data bytes. It is important to also note that this value could overflow the 1 BYTE size, and thus has 2 bytes to 
describe it, a HIGH BYTE and LOW BYTE. 

Adding all these values together produces the number 337. To put this into the data frame we will need to break it 
into its high byte and low byte values. The binary Representation of 337 is `0000 0001 0101 0001`


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
 
