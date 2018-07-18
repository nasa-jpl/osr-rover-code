import time
from rover import Robot
from connections import Connections

class Rover(Robot, Connections):
	def __init__(	self, 
					config, 
					bt_flag = 0, 
					xbox_flag = 0,
					unix_flag = 0
				):
		
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
		try:
			v,r = self.getDriveVals()
			if v,r != self.prev_cmd:
				self.sendCommands(v,r)
				self.prev_cmd = v,r

		except KeyboardInterrupt:
			self.cleanup()

		except Exception as e:
			print e
			self.cleanup()
			time.sleep(0.5)
			self.connectController()

		if self.unix_flag:
			try:
				self.sendUnixData()
			except Exception as e:
				print e
				self.unix_flag = 0
			
			
	def cleanup(self):
		self.killMotors()
		self.closeConnections()

	
		
