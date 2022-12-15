import INA260
import RPi.GPIO as gpio

pm = INA260.INA260(INA260.INA260_I2CADDR_A0A1)

gpio.setmode(gpio.BCM)
gpio.setup(27, gpio.IN, gpio.PUD_UP)
print(gpio.input(27))
print("Address: 0x%X" % (pm.getAddress()))

#GPI27 will go to zero during an alarm. 

#print("Read Mask 0x%X" % (pm.readMaskEnable()))
print("Alert Limit 0x%X" % (pm.readLimitRegister()))
print(gpio.input(27))

print("Initial Configuration Done\n")

#pm.setUnderVoltageLimit(40.96-0.00125)
pm.setUnderVoltageLimit(12)

i = 0

if pm:
	while(i < 3): 
		print("Voltage: %2.4f" % (pm.readVoltage()))
		print("Current: %2.4f" % (pm.readCurrent()))
		print("Power: %2.4f" % (pm.readPower()))
		print("Alarm %d" % (gpio.input(27)))
		limit = pm.readLimitRegister()
		print("Alert Limit 0x%X, %2.4f" % (limit, limit*0.00125))
#		print("Mask 0x%X\n" % (pm.readMaskEnable()))
		i += 1
		
else:
	print("Failed to initialize INA260")
	
	
