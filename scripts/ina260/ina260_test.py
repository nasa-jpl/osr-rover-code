"""
Basic test script for the ina260 on the motor board

Copied this over with minor changes from David Schooley's repo at
 https://github.com/dcschooley/ina260

Instructions:

1. test i2c

install i2c-tools
 $ sudo apt-get install i2c-tools
then run i2cdetect to ensure an i2c device is found:
 $ sudo i2cdetect 1
when I ran this on the rpi on brain board, I saw a little table with the default 
 i2c address of the ina260, 0x40, displayed as present. Confirms the device is 
 detectable over i2c

2. add user to i2c group

add user to i2c group
 $ sudo adduser <username> i2c
then log out (close terminal) and log back in to have it take effect
test that you're in the i2c group by running:
 $ groups
you should see `i2c` in the output list

3. run the script!
python3 ina260_test.py

output should look like (note that voltage is 0.0000V because the ina260
 is mounted in series with battery voltage on the motor board, for current 
 measurement):
```
    Found INA260
    Voltage: 0.0000
    Current: 0.3950
    Configuration: 0x6127
    Mode: 0x3
    Post-reset Configuration: 0x6127
    Average Mode: 0x0
    Average Mode: 0x2
    Configuration: 0x6527
    Voltage Conversion: 0x4
    Voltage Conversion: 0x5
    Current Conversion: 0x4
    Current Conversion: 0x3
    Voltage Conversion: 0x7
    Current Conversion: 0x7
    Mask/enable: 0x0
    Limit register: 0x0
    Mask/enable: 0x0
    Limit register: 0x2580
```
"""

import ina260
import RPi.GPIO as gpio

# default address out of the box
# ADDRESS=ina260.INA260_I2CADDR_DEFAULT
# use 0x45 if solder applied to both A0 and A1
ADDRESS=ina260.INA260_I2CADDR_A0A1

pm = ina260.INA260(addr=ADDRESS)

## Commenting out gpio alert pin stuff for now
# gpio.setmode(gpio.BCM)
# gpio.setup(4, gpio.IN, gpio.PUD_UP)
# print(gpio.input(4))

#GPIO4 will go to zero during an alarm. 


if pm:

    correctDevice = pm.checkCorrectDevice()
    if correctDevice:
        print ("Found INA260")
    else:
        print ("Oops!")

    # a lot of times, this voltage measurement comes out wrong (0.00)
    print("Voltage: %2.4f" % (pm.readVoltage()))
    print("Current: %2.4f" % (pm.readCurrent()))
    conf = pm.readConfig()
    print ("Configuration: 0x%X" % (conf))
    pm.setModeTriggered()
    print("Mode: 0x%X" % (pm.readMode()))
    pm.resetConfig()
    print("Post-reset Configuration: 0x%X" % (pm.readConfig()))
    
    avgMode = pm.readAvgMode()
    print("Average Mode: 0x%X" % (avgMode))
    
    
    pm.setAvgMode_16()
    avgMode = pm.readAvgMode()
    print("Average Mode: 0x%X" % (avgMode))

    conf = pm.readConfig()
    print("Configuration: 0x%X" % (conf))
    
    voltConv = pm.readVoltCnvTime()
    print("Voltage Conversion: 0x%X" % (voltConv))
    pm.setVoltCnvTime_2_116ms()
    voltConv = pm.readVoltCnvTime()
    print("Voltage Conversion: 0x%X" % (voltConv))

    curConv = pm.readCurrCnvTime()
    print("Current Conversion: 0x%X" % (curConv))
    pm.setCurrCnvTime_588usec()
    curConv = pm.readCurrCnvTime()
    print("Current Conversion: 0x%X" % (curConv))
    
    pm.setCnvTime_8_244ms()
    voltConv = pm.readVoltCnvTime()
    print("Voltage Conversion: 0x%X" % (voltConv))
    curConv = pm.readCurrCnvTime()
    print("Current Conversion: 0x%X" % (curConv))
    
    # voltage limits don't work well, because it doesn't seem to be measuring voltage correctly, at least on my board
    # pm.setUnderVoltageLimit(10.0)
    # pm.setOverVoltageLimit(17.0)

    # 0.40 should trigger the alert signal and LED
    pm.setOverCurrentLimit(0.40)
    # 0.60 should not
    # pm.setOverCurrentLimit(0.60)
    limit = pm.readLimitRegister()
    print ("Limit register: 0x%X" % (limit))
    
    alert = pm.getAlarmState()
    if alert:
        print("ALARM!\n")
    else:
        print("No Alarm\n")

    # pm.setAlertPolarity(1)
    # pm.setAlertLatch()


    mask = pm.readMaskEnable()
    print ("Mask/enable: 0x%X" % (mask))
    limit = pm.readLimitRegister()
    print ("Limit register: 0x%X" % (limit))
    pm.setUnderVoltageLimit(12.0)
#	mask = pm.readMaskEnable()
    print ("Mask/enable: 0x%X" % (mask))
    limit = pm.readLimitRegister()
    print ("Limit register: 0x%X" % (limit))

    # print "Alarm:", gpio.input(4)

        
else:
    print("Failed to initialize INA260")
    
    
