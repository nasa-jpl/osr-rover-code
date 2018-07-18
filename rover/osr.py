#!/usr/bin/env python
from robot import Robot
from connections import Connections

class Rover(Robot, Connections):
	'''
	Rover class manages connections from the sockets and remote devices 
	
	and sends signals to the robot to drive
	
	:param class object Robot      : Robot class to do all calculations to move robot
	:param class object Connections: Connections class manages sockets and remote device connections
	
	'''
	def __init__( self, config, bt_flag = 0, xbox_flag = 0, unix_flag = 0 ):
		'''
		The init takes in the config file, and flags to tell which way to control the robot
		
		:param Json file config : Stores the configurations for the robot
		:param int bt_flag      : flag to turn on bluetooth listener
		:param int xbox_flag    : flag to turn on xbox listener
		:param int unix_flag    : flag to open unix socket for LED screen
		
		'''

		self.bt_flag = bt_flag
		self.xbox_flag = xbox_flag
		self.unix_flag = unix_flag
		
		super(Rover,self).__init__(config)
		self.prev_cmd = [None,None]
		
		if bt_flag and xbox_flag:
			raise Exception( "[Rover init] Cannot initialize with both bluetooth and Xbox, run with only one argument")
		elif bt_flag:   self.connection_type = "b"
		elif xbox_flag: self.connection_type = "x"
			
		self.connectController()
	
	def drive(self):
		'''
		Takes drive commands and sends them to the robot
		
		'''
		cmds = self.getDriveVals()
		if cmds != self.prev_cmd:                   #if no new command don't send commands to robot
			self.sendCommands(cmds[0],cmds[1])
			self.prev_cmd = cmds
		else:										#still monitor the corners to reduce jitter though
			self.cornerPosControl(self.calculateTargetDeg(cmds[1]))

		if self.unix_flag: self.sendFaceCmd()

	def sendFaceCmd(self):
		'''
		Attempts to send commands to the LED screen over unix socket
		
		'''
		try:
			self.sendUnixData()
		except Exception as e:
			print e
			self.unix_flag = 0
				
	def cleanup(self):
		'''
		Cleanup method for closing connections, and stopping the motors
		
		'''
		self.killMotors()
		self.closeConnections()

	
		
