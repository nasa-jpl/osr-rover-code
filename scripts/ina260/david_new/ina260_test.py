import INA260
import RPi.GPIO as gpio

pm = INA260.INA260(INA260.INA260_I2CADDR_DEFAULT)

gpio.setmode(gpio.BCM)
gpio.setup(27, gpio.IN, gpio.PUD_UP)
print(gpio.input(27))

#GPIO4 will go to zero during an alarm. 


if pm:

    correctDevice = pm.checkCorrectDevice()
    if correctDevice:
        print ("Found INA260")
    else:
        print ("Oops!")

    print("Voltage: %2.4f" % (pm.readVoltage()))
    conf = pm.readConfig()
    print ("Configuration: 0x%X" % (conf))
#	pm.setModeTriggered()
    print("Mode: 0x%X" % (pm.readMode()))
    pm.resetConfig()
    print("Post-reset Configuration: 0x%X" % (pm.readConfig()))
    print("Voltage: %2.4f" % (pm.readVoltage()))	
    avgMode = pm.readAvgMode()
    print("Average Mode: 0x%X" % (avgMode))
    
    
#	pm.setAvgMode_16()
    avgMode = pm.readAvgMode()
    print("Average Mode: 0x%X" % (avgMode))

    conf = pm.readConfig()
    print("Configuration: 0x%X" % (conf))
    
    voltConv = pm.readVoltCnvTime()
    print("Voltage Conversion: 0x%X" % (voltConv))
#	pm.setVoltCnvTime_2_116ms()
    voltConv = pm.readVoltCnvTime()
    print("Voltage Conversion: 0x%X" % (voltConv))

    curConv = pm.readCurrCnvTime()
    print("Current Conversion: 0x%X" % (curConv))
#	pm.setCurrCnvTime_588usec()
    curConv = pm.readCurrCnvTime()
    print("Current Conversion: 0x%X" % (curConv))
    
    pm.setCnvTime_8_244ms()
    voltConv = pm.readVoltCnvTime()
    print("Voltage Conversion: 0x%X" % (voltConv))
    curConv = pm.readCurrCnvTime()
    print("Current Conversion: 0x%X" % (curConv))
    print("Resetting configuration")
    pm.resetConfig()	
#	pm.setCnvTime_8_244ms()
    limit = pm.readLimitRegister()
    print ("Limit register: 0x%X" % (limit))
#	pm.setOverVoltageLimit(17.0)
# GPIO4 will go to zero during an alarm. 
#	pm.setAlertPolarity(1)
#	pm.setAlertLatch()
    print("Alarm:", gpio.input(27))

    mask = pm.readMaskEnable()
    print ("Mask/enable: 0x%X" % (mask))
    limit = pm.readLimitRegister()
    print ("Limit register: 0x%X" % (limit))
    pm.setUnderVoltageLimit(12.0)
    print("Setting under voltage limit")
#	mask = pm.readMaskEnable()
    print ("Mask/enable: 0x%X" % (mask))
    limit = pm.readLimitRegister()
    print ("Limit register: 0x%X" % (limit))

    print("Alarm:", gpio.input(27))
    print("Voltage: %2.4f" % (pm.readVoltage()))
    print("Current: %2.4f" % (pm.readCurrent()))
    print("Power: %2.4f" % (pm.readPower()))
    print("Voltage: %2.4f" % (pm.readVoltage()))
        
else:
    print("Failed to initialize INA260")
    
    
