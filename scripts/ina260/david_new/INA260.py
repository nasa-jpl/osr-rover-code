import smbus
import time

# This code is derived from the Arduino code developed by Adafruit 
# along with the TI documentation to the INA260 IC.

# The byte order coming out of the INA260 is reversed from normal SMbus or I2C
# There is a lot of byte switching down below. 

# Creating an INA260 object and then calling the member functions
#	readVoltage()
#	readCurrent()
#	readPower()
# should be enough to get started. 
# The alarm functions set alarm types and can only be called once. 
# In order for the alarm to work on the Pi, the pull-up for whatever GPIO
# you are using needs to be set to pull-up using gpio.PUD_UP. 
# The alarm pin will be normally high and will go low during an alarm condition. 
# The alarm functions can only be set once. Changing the alarm function requires
# resetting the chip. The reset requirement is specific to this interface module. 


# The initialization code sets the INA260 to continuous sampling in case
# it was previously disabled. This is the default mode. 
# Calling the member function setModeShutdown() will shut down the 
# board to reduce current usage if not in use. 

def bswap(val):
	ret = ((val>>8) & 0xff) + ((val & 0xff)<<8)
	return ret
	
# The I2C address can be changed by bridging the address pads on the Adafruit circuit 
# board. The addresses below are untested. 
INA260_I2CADDR_DEFAULT = 0x40 	# INA260 default i2c address
INA260_I2CADDR_A0 = 0x41		# A0 connected to Vcc
INA260_I2CADDR_A1 = 0x44		# A1 connected to Vcc
INA260_I2CADDR_A0A1 = 0x45		# A0 and A1 connected to Vcc

# Contants for talking to the INA260
INA260_REG_CONFIG = 0x00      	# Configuration register
INA260_REG_CURRENT = 0x01 		# Current measurement register (signed) in mA
INA260_REG_BUSVOLTAGE = 0x02 	# Bus voltage measurement register in mV
INA260_REG_POWER = 0x03      	# Power calculation register in mW
INA260_REG_MASK_ENABLE = 0x06 	# Was 0x06 Interrupt/Alert setting and checking register
INA260_REG_ALERT_LIMIT = 0x07 	# Alert limit value register
INA260_REG_MFG_UID = 0xFE     	# Manufacturer ID Register
INA260_REG_DIE_UID = 0xFF     	# Die ID and Revision Register


INA260_MODE_SHUTDOWN = 0x00 	# SHUTDOWN: Minimize quiescient current and
#                                  turn off current into the device inputs. Set
#                                  another mode to exit shutown mode **/
INA260_MODE_TRIGGERED = 0x03 	# TRIGGERED: Trigger a one-shot measurement
#                                      of current and bus voltage. Set the TRIGGERED
#                                      mode again to take a new measurement **/
INA260_MODE_CONTINUOUS = 0x07 # CONTINUOUS: (Default) Continuously update

INA260_TIME_140_us = 0x0 # Measurement time: 140us
INA260_TIME_204_us = 0x1 # Measurement time: 204us
INA260_TIME_332_us = 0x2 # Measurement time: 332us
INA260_TIME_558_us = 0x3 # Measurement time: 558us
INA260_TIME_1_1_ms = 0x4 # Measurement time: 1.1ms (Default)
INA260_TIME_2_116_ms = 0x5 # Measurement time: 2.116ms
INA260_TIME_4_156_ms = 0x6 # Measurement time: 4.156ms
INA260_TIME_8_244_ms = 0x7 # Measurement time: 8.244ms

INA260_COUNT_1 = 0x0 # Window size: 1 sample (Default)
INA260_COUNT_4 = 0x1 # Window size: 4 samples
INA260_COUNT_16 = 0x2 # Window size: 16 samples
INA260_COUNT_64 = 0x3 # Window size: 64 samples
INA260_COUNT_128 = 0x4 # Window size: 128 samples
INA260_COUNT_256 = 0x5 # Window size: 256 samples
INA260_COUNT_512 = 0x6 # Window size: 512 samples
INA260_COUNT_1024 = 0x7 # Window size: 1024 samples

voltageScale = 0.00125
currentScale = 0.00125
powerScale = 0.01

ALARM_NOTSET = 0
ALARM_HIGHVOLTAGE = 1
ALARM_LOWVOLTAGE = 2
ALARM_HIGHCURRENT = 3
ALARM_LOWCURRENT = 4
ALARM_OVERPOWER = 5

class INA260(object):

	_myAddr = INA260_I2CADDR_DEFAULT

	_myBus = None
	_myAlarmType = ALARM_NOTSET

	def __init__(self, addr=INA260_I2CADDR_DEFAULT):
		self._myAddr = addr
		self._myBus = smbus.SMBus(1) 
		try:
			# We would like to call the checkCorrectDevice member function here
			# but Python won't let us. 
			reg = self._myBus.read_word_data(addr, INA260_REG_MFG_UID)
			if reg != 0x4954:
				raise Exception("Wrong type of part at address 0x%x" % (addr))
			self._myBus.write_word_data(addr, INA260_REG_CONFIG, 0x80)
		except:
			self._myBus = None

	def _readAnalog(self, param, scale):
		if self._myBus:
			analog = self._myBus.read_word_data(self._myAddr, param)
			analog = bswap(analog) * scale
		else:
			analog = None
		return(analog)
		
	def _writeAnalog(self, param, analog, scale):
		if self._myBus:
			analog = int(analog / scale)
			analog = bswap(analog)
			self._myBus.write_word_data(self._myAddr, param, analog)
			
	def _readDigital(self, param):
		if self._myBus:
			dig = self._myBus.read_word_data(self._myAddr, param)
			dig = bswap(dig)
		else:
			dig = None
		return(dig)

	def _writeDigital(self, param, dig):
		if self._myBus:
			dig = bswap(dig)
			self._myBus.write_word_data(self._myAddr, param, dig)

	def _setOpMode(self, mode):
		if self._myBus:
			reg = self._myBus.read_word_data(self._myAddr, INA260_REG_CONFIG)
			reg = bswap(reg)
			reg = (reg & 0xFFF0) + mode
			reg = bswap(reg)
			self._myBus.write_word_data(self._myAddr, INA260_REG_CONFIG, reg)
		
	def _setAvgMode(self, mode):
		if self._myBus:
			reg = self._myBus.read_word_data(self._myAddr, INA260_REG_CONFIG)
			reg = bswap(reg)
			reg = (reg & 0xF1FF) + (mode << 9)	
			reg = bswap(reg)
			self._myBus.write_word_data(self._myAddr, INA260_REG_CONFIG, reg)
		
	def _setCurrCnvTime(self, time):
		if self._myBus:
			reg = self._myBus.read_word_data(self._myAddr, INA260_REG_CONFIG)
			reg = bswap(reg)
			reg = (reg & 0xFFC7) + (time << 3)
			reg = bswap(reg)
			self._myBus.write_word_data(self._myAddr, INA260_REG_CONFIG, reg)

	def _setVoltCnvTime(self, time):
		if self._myBus:
			reg = self._myBus.read_word_data(self._myAddr, INA260_REG_CONFIG)
			reg = bswap(reg)
			reg = (reg & 0xFE3F) + (time << 6)
			reg = bswap(reg)
			self._myBus.write_word_data(self._myAddr, INA260_REG_CONFIG, reg)


		
### Public methods
		
	def getAddress(self):
		return(self._myAddr)

# Methods to read the voltage, current, and power			
	def readVoltage(self):
		voltage = self._readAnalog(INA260_REG_BUSVOLTAGE, voltageScale)
		return(voltage)

	def readCurrent(self):
		current = self._readAnalog(INA260_REG_CURRENT, currentScale)
		return(current)
		
	def readPower(self):
		power = self._readAnalog(INA260_REG_POWER, powerScale)
		return(power)

	def readMode(self):
		mode = self._readDigital(INA260_REG_CONFIG) & 0x7
		return(mode)	
		
	def readAvgMode(self):
		mode = self._readDigital(INA260_REG_CONFIG)
		mode = (mode >> 9) & 0x7
		return(mode)
		
	def readVoltCnvTime(self):
		mode = self._readDigital(INA260_REG_CONFIG)
		mode = (mode >> 6) & 0x7
		return(mode)		

	def readCurrCnvTime(self):
		mode = self._readDigital(INA260_REG_CONFIG)
		mode = (mode >> 3) & 0x7
		return(mode)		
	
	def readConfig(self):
		reg = self._readDigital(INA260_REG_CONFIG)
		return(reg)
		
	def resetConfig(self):
		self._writeDigital(INA260_REG_CONFIG, 0x8000)
		self._myAlarmType = ALARM_NOTSET
		
	def setModeShutdown(self):
		self._setOpMode(INA260_MODE_SHUTDOWN)
		
	def setModeTriggered(self):
		self._setOpMode(INA260_MODE_TRIGGERED)

	def setModeContinuous(self):
		self._setOpMode(INA260_MODE_CONTINUOUS)
		
		
# Functions to set the averaging mode in terms of the number of samples
	def setAvgMode_1(self):
		self._setAvgMode(INA260_COUNT_1)

	def setAvgMode_4(self):
		self._setAvgMode(INA260_COUNT_4)

	def setAvgMode_16(self):
		self._setAvgMode(INA260_COUNT_16)

	def setAvgMode_64(self):
		self._setAvgMode(INA260_COUNT_64)

	def setAvgMode_128(self):
		self._setAvgMode(INA260_COUNT_128)

	def setAvgMode_256(self):
		self._setAvgMode(INA260_COUNT_256)

	def setAvgMode_512(self):
		self._setAvgMode(INA260_COUNT_512)

	def setAvgMode_1024(self):
		self._setAvgMode(INA260_COUNT_1024)
		
		
# Functions to set the conversion times for current or voltage measurements. 
# Functions with 'Curr' or 'Volt' in the name set the conversion times for
# current or voltage, respectively. Functions without 'Curr' or 'Volt' in the 
# name are convenience functions that set both parameters to be the same. 
	def setCurrCnvTime_140usec(self):
		self._setCurrCnvTime(INA260_TIME_140_us)

	def setVoltCnvTime_140usec(self):
		self._setVoltCnvTime(INA260_TIME_140_us)
		
	def setCnvTime_140usec(self):
		self.setCurrCnvTime_140usec()
		self.setVoltCnvTime_140usec() 
		
	def setCurrCnvTime_204usec(self):
		self._setCurrCnvTime(INA260_TIME_204_us)

	def setVoltCnvTime_204usec(self):
		self._setVoltCnvTime(INA260_TIME_204_us)
		
	def setCnvTime_204usec(self):
		self.setCurrCnvTime_204usec()
		self.setVoltCnvTime_204usec() 
		
	def setCurrCnvTime_332usec(self):
		self._setCurrCnvTime(INA260_TIME_332_us)

	def setVoltCnvTime_332usec(self):
		self._setVoltCnvTime(INA260_TIME_332_us)
		
	def setCnvTime_332usec(self):
		self.setCurrCnvTime_332usec()
		self.setVoltCnvTime_332usec() 

	def setCurrCnvTime_588usec(self):
		self._setCurrCnvTime(INA260_TIME_558_us)

	def setVoltCnvTime_588usec(self):
		self._setVoltCnvTime(INA260_TIME_588_us)
		
	def setCnvTime_588usec(self):
		self.setCurrCnvTime_588usec()
		self.setVoltCnvTime_588usec() 

	def setCurrCnvTime_1_1ms(self):
		self._setCurrCnvTime(INA260_TIME_1_1_ms)

	def setVoltCnvTime_1_1ms(self):
		self._setVoltCnvTime(INA260_TIME_1_1_ms)
		
	def setCnvTime_1_1ms(self):
		self.setCurrCnvTime_1_1ms()
		self.setVoltCnvTime_1_1ms() 

	def setCurrCnvTime_2_116ms(self):
		self._setCurrCnvTime(INA260_TIME_2_116_ms)

	def setVoltCnvTime_2_116ms(self):
		self._setVoltCnvTime(INA260_TIME_2_116_ms)
		
	def setCnvTime_2_116ms(self):
		self.setCurrCnvTime_2_116ms()
		self.setVoltCnvTime_2_116ms() 

	def setCurrCnvTime_4_156ms(self):
		self._setCurrCnvTime(INA260_TIME_4_156_ms)

	def setVoltCnvTime_4_156ms(self):
		self._setVoltCnvTime(INA260_TIME_4_156_ms)
		
	def setCnvTime_4_156ms(self):
		self.setCurrCnvTime_4_156ms()
		self.setVoltCnvTime_4_156ms() 

	def setCurrCnvTime_8_244ms(self):
		self._setCurrCnvTime(INA260_TIME_8_244_ms)

	def setVoltCnvTime_8_244ms(self):
		self._setVoltCnvTime(INA260_TIME_8_244_ms)
		
	def setCnvTime_8_244ms(self):
		self.setCurrCnvTime_8_244ms()
		self.setVoltCnvTime_8_244ms() 
		
# Functions to set alarm limits		
# These functions are a one shot deal.
# Call resetConfig to completely reset the configuration and allow a change
# of the alarm. 

	def getAlarmType():
		return self._myAlarmType

	def setOverCurrentLimit(self, limit):
		if self._myBus and (self._myAlarmType == ALARM_NOTSET):
			self._writeAnalog(INA260_REG_ALERT_LIMIT, limit, currentScale)
			mask = self._readDigital(INA260_REG_MASK_ENABLE)
			mask = (mask & 0x0403) + 0x8000
			self._writeDigital(INA260_REG_MASK_ENABLE, mask)
			self._myAlarmType = ALARM_HIGHCURRENT

	def setUnderCurrentLimit(self, limit):
		if self._myBus and (self._myAlarmType == ALARM_NOTSET):
			self._writeAnalog(INA260_REG_ALERT_LIMIT, limit, currentScale)
			mask = self._readDigital(INA260_REG_MASK_ENABLE)
			mask = (mask & 0x0403) + 0x4000
			self._writeDigital(INA260_REG_MASK_ENABLE, mask)
			self._myAlarmType = ALARM_LOWCURRENT

	def setOverVoltageLimit(self, limit):
		if self._myBus and (self._myAlarmType == ALARM_NOTSET):
			self._writeAnalog(INA260_REG_ALERT_LIMIT, limit, voltageScale)
			mask = self._readDigital(INA260_REG_MASK_ENABLE)
			mask = (mask & 0x0403) + 0x2000
			self._writeDigital(INA260_REG_MASK_ENABLE, mask)
			self._myAlarmType = ALARM_HIGHVOLTAGE

	def setUnderVoltageLimit(self, limit):
		if self._myBus and (self._myAlarmType == ALARM_NOTSET):
			self._writeAnalog(INA260_REG_ALERT_LIMIT, limit, voltageScale)
			mask = self._readDigital(INA260_REG_MASK_ENABLE)
			mask = (mask & 0x0403) + 0x1000
			self._writeDigital(INA260_REG_MASK_ENABLE, mask)
			self._myAlarmType = ALARM_LOWVOLTAGE

	def setOverPowerLimit(self, limit):
		if self._myBus and (self._myAlarmType == ALARM_NOTSET):
			self._writeAnalog(INA260_REG_ALERT_LIMIT, limit, powerScale)
			mask = self._readDigital(INA260_REG_MASK_ENABLE)
			mask = (mask & 0x0403) + 0x800
			self._writeDigital(INA260_REG_MASK_ENABLE, mask)
			self._myAlarmType = ALARM_OVERPOWER
			
	def setAlertLatch(self):
		dig = self._readDigital(INA260_REG_MASK_ENABLE)
		if dig:
			dig = (dig & 0xFFFE) + 1
			self._writeDigital(INA260_REG_MASK_ENABLE, dig)

	def clearAlertLatch(self):
		dig = self._readDigital(INA260_REG_MASK_ENABLE)
		if dig:
			dig = dig & 0xFFFE
			self._writeDigital(INA260_REG_MASK_ENABLE, dig)
			
	def setAlertPolarity(self, pol):
		dig = self._readDigital(INA260_REG_MASK_ENABLE)
		if dig:
			if pol:
				dig = (dig & 0xFFFD) + 2
			else:
				dig = dig & 0xFFFD
			self._writeDigital(INA260_REG_MASK_ENABLE, dig)
			
	def getAlarmState(self):
		dig = self._readDigital(INA260_REG_MASK_ENABLE) & 0x10
		if dig:
			return True
		else:
			return False

# These are public functions but they are only useful for testing the API. 	
	def readMaskEnable(self):
		dig = self._readDigital(INA260_REG_MASK_ENABLE)
		return (dig)
	
	def readLimitRegister(self):
		dig = self._readDigital(INA260_REG_ALERT_LIMIT)
		return (dig)

	def readManufID(self):
		manufID = self._readDigital(INA260_REG_MFG_UID)
		return manufID

	def checkCorrectDevice(self):
		manufID = self.readManufID()
		if manufID == 0x5449:
			return(True)
		else:
			print("Invalid ID:", manufID)
			return(False)
			

