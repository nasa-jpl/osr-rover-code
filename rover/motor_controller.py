#!/usr/bin/env python
from roboclaw import Roboclaw
import time
import serial
import math

class Motor(object):
	'''
	Motor class contains the methods necessary to send commands to the motor controllers
	
	for the corner and drive motors. There are many other ways of commanding the motors
	
	from the RoboClaw, we suggest trying to write your own Closed loop feedback method for
	
	the drive motors!

	'''
	def __init__(self,config):
		super(Motor,self).__init__(config)
			
		self.rc = Roboclaw( config['CONTROLLER_CONFIG']['device'],
							config['CONTROLLER_CONFIG']['baud_rate']
							)
		self.rc.Open()
		
		self.address         = config['MOTOR_CONFIG']['controller_address']
		self.accel           = [0]    * 10
		self.qpps            = [None] * 10
		self.err             = [None] * 5

		version = 1
		for address in self.address:
			version = version & self.rc.ReadVersion(address)[0]

		if version != 0:
			print "[Motor__init__] Sucessfully connected to RoboClaw motor controllers"
		else:
			raise Exception("Unable to establish connection to Roboclaw motor controllers")

		self.enc_min =[]
		self.enc_max =[]
		for address in self.address: 
			self.rc.SetMainVoltages(address,
									int(config['BATTERY_CONFIG']['low_voltage']*10), 
									int(config['BATTERY_CONFIG']['high_voltage']*10)
									)
			if address == 131 or address == 132:
				self.rc.SetM1MaxCurrent(address, int(config['MOTOR_CONFIG']['max_corner_current']*100))
				self.rc.SetM2MaxCurrent(address, int(config['MOTOR_CONFIG']['max_corner_current']*100))

				self.enc_min.append(self.rc.ReadM1PositionPID(address)[-2])
				self.enc_min.append(self.rc.ReadM2PositionPID(address)[-2])
				self.enc_max.append(self.rc.ReadM1PositionPID(address)[-1])
				self.enc_max.append(self.rc.ReadM2PositionPID(address)[-1])
				
			else:
				self.rc.SetM1MaxCurrent(address, int(config['MOTOR_CONFIG']['max_drive_current']*100))
				self.rc.SetM2MaxCurrent(address, int(config['MOTOR_CONFIG']['max_drive_current']*100))
				self.rc.ResetEncoders(address)

		for address in self.address:
			self.rc.WriteNVM(address)
			
		for address in self.address:
			self.rc.ReadNVM(address)

		voltage = self.rc.ReadMainBatteryVoltage(0x80)[1]/10.0
		if voltage >= config['BATTERY_CONFIG']['low_voltage']:
			print "[Motor__init__] Voltage is safe at: ",voltage, "V"
		else:
			raise Exception("Unsafe Voltage of" + voltage + " Volts") 

		i = 0

		for address in self.address:
			self.qpps[i]    = self.rc.ReadM1VelocityPID(address)[4]
			self.accel[i]   = int(self.qpps[i]*2)
			self.qpps[i+1]  = self.rc.ReadM2VelocityPID(address)[4]
			self.accel[i+1] = int(self.qpps[i]*2)
			i+=2
		
		self.errorCheck()

	def cornerToPosition(self,tick):
		'''
		Method to send position commands to the corner motor

		:param list tick: A list of ticks for each of the corner motors to
		move to, if tick[i] is 0 it instead stops that motor from moving

		'''
		speed, accel = 1000,2000            #These values could potentially need tuning still
		self.errorCheck()
		for i in range(4):
			index = int(math.ceil((i+1)/2.0)+2)
			if tick[i]:
				if (i % 2):  self.rc.SpeedAccelDeccelPositionM2(self.address[index],accel,speed,accel,tick[i],1)
				else:        self.rc.SpeedAccelDeccelPositionM1(self.address[index],accel,speed,accel,tick[i],1)				
			else:
				if not (i % 2): self.rc.ForwardM1(self.address[index],0)
				else:           self.rc.ForwardM2(self.address[index],0)
			
	def sendMotorDuty(self, motorID, speed):
		'''
		Wrapper method for an easier interface to control the drive motors,
		
		sends open-loop commands to the motors

		:param int motorID: number that corresponds to each physical motor
		:param int speed: Speed for each motor, range from 0-127

		'''
		self.errorCheck()
		addr = self.address[int(motorID/2)]
		if speed > 0: 
			if not motorID % 2: command = self.rc.ForwardM1
			else:               command = self.rc.ForwardM2
		else: 
			if not motorID % 2: command = self.rc.BackwardM1
			else:               command = self.rc.BackwardM2

		speed = abs(int(speed * 127))
		
		return command(addr,speed)

	def killMotors(self):
		'''
		Stops all motors on Rover
		'''
		for i in range(5):
			self.rc.ForwardM1(self.address[i],0)
			self.rc.ForwardM2(self.address[i],0)
		
	def errorCheck(self):
		'''
		Checks error status of each motor controller, returns 0 if any errors occur
		'''

		for i in range(5):
			self.err[i] = self.rc.ReadError(self.address[i])[1]
		for error in self.err:
			if error:
				self.killMotors()
				self.writeError()
				raise Exception("Motor controller Error", error)
		return 1

	def writeError(self):
		'''
		Writes the list of errors to a text file for later examination
		'''

		f = open('errorLog.txt','a')
		errors = ','.join(str(e) for e in self.err)
		f.write('\n' + 'Errors: ' + '[' + errors + ']' + ' at: ' + str(datetime.datetime.now()))
		f.close()
