#!/usr/bin/env python

import time
import serial
import math
import rospy

from roboclaw import Roboclaw

from osr_msgs.msg import Commands, Encoder, Status


class RoboclawWrapper(object):
	"""Interface between the roboclaw motor drivers and the higher level rover code"""

	def __init__(self):
		rospy.loginfo( "Initializing motor controllers")

		# initialize attributes
		self.rc = None
		self.accel = [0] * 10
		self.qpps = [None] * 10
		self.err = [None] * 5
		self.address = []
		self.enc_min = []
		self.enc_max = []
		self.enc = [None]*4
		self.mutex = False

		self.establish_roboclaw_connections()
		self.killMotors()  # don't move at start
		self.setup_encoders()

		# save settings to non-volatile (permanent) memory
		for address in self.address:
			self.rc.WriteNVM(address)

		for address in self.address:
			self.rc.ReadNVM(address)

		index = 0
		for address in self.address:
			self.qpps[index] = self.rc.ReadM1VelocityPID(address)[4]
			self.accel[index] = int(self.qpps[index]*2)
			self.qpps[index+1]  = self.rc.ReadM2VelocityPID(address)[4]
			self.accel[index+1] = int(self.qpps[index]*2)
			index+=2

		accel_max = 655359
		accel_rate = 0.5
		self.accel_pos = int((accel_max /2) + accel_max * accel_rate)
		self.accel_neg = int((accel_max /2) - accel_max * accel_rate)
		self.errorCheck()
		mids = [None]*4

		# calculate middle for corner motors based on max and min values read from the roboclaws
		for i in range(4):
			mids[i] = (self.enc_max[i] + self.enc_min[i])/2
		#self.cornerToPosition(mids)
		time.sleep(2)
		self.killMotors()

		# set up publishers and subscribers
		self.cmd_sub = rospy.Subscriber("/robot_commands", Commands, self.robot_cmd_cb)
		self.enc_pub = rospy.Publisher("/encoder", Encoder, queue_size=1)
		self.status_pub = rospy.Publisher("/status", Status, queue_size=1)

	def run(self):
		"""Blocking loop which runs after initialization has completed"""
		rate = rospy.Rate(5)

		status = Status()
		enc = Encoder()

		enc.abs_enc = [1000] * 4
		enc.abs_enc_angles = [-100] * 4
		status.battery = 0
		status.temp = [0] * 5
		status.current = [0] * 10
		status.error_status = [0] * 5

		counter = 0
		while not rospy.is_shutdown():

			while self.mutex and not rospy.is_shutdown():
				rate.sleep()
			self.mutex = True

			enc.abs_enc = self.getCornerEnc()
			# mc_data.abs_enc_angles = self.getCornerEncAngle()
			if (counter >= 10):
				status.battery = self.getBattery()
				status.temp = self.getTemp()
				status.current = self.getCurrents()
				status.error_status = self.getErrors()
				self.status_pub.publish(status)
				counter = 0

			self.mutex = False
			self.enc_pub.publish(enc)
			counter += 1
			rate.sleep()

	def robot_cmd_cb(self, cmds):
		r = rospy.Rate(10)
		rospy.logdebug("Robot command callback received: {}".format(cmds))

		while self.mutex and not rospy.is_shutdown():
			r.sleep()

		self.mutex = True
		self.cornerToPosition(cmds.corner_motor)

		for i in range(6):
			# PUT THIS BACK IN
			# self.sendMotorDuty(i,cmds.drive_motor[i])
			self.sendSignedDutyAccel(i, cmds.drive_motor[i])
			pass
		self.mutex = False

	def establish_roboclaw_connections(self):
		"""
		Attempt connecting to the roboclaws

		:raises Exception: when connection to one or more of the roboclaws is unsuccessful
		"""
		self.rc = Roboclaw(rospy.get_param('motor_controller_device', "/dev/serial0"),
						   rospy.get_param('baud_rate', 115200))
		self.rc.Open()

		address_raw = rospy.get_param('motor_controller_addresses')
		address_list = (address_raw.split(','))
		self.address = [None]*len(address_list)
		for i in range(len(address_list)):
			self.address[i] = int(address_list[i])

		# initialize connection status to successful
		all_connected = True
		for address in self.address:
			rospy.logdebug("Attempting to talk to motor controller ''".format(address))
                        version_response = self.rc.ReadVersion(address)
			connected = bool(version_response[0])
			if not connected:
				rospy.logerr("Unable to connect to roboclaw at '{}'".format(address))
				all_connected = False
			else:
				rospy.logdebug("Roboclaw version for address '{}': '{}'".format(address, version_response[1]))
		if all_connected:
			rospy.loginfo("Sucessfully connected to RoboClaw motor controllers")
		else:
			raise Exception("Unable to establish connection to one or more of the Roboclaw motor controllers")

	def setup_encoders(self):
		"""Set up the encoders for use in init"""
		for address in self.address:
			#self.rc.SetMainVoltages(address, rospy.get_param('battery_low', 11)*10), rospy.get_param('battery_high', 18)*10))

			if address == 131 or address == 132:
				#self.rc.SetM1MaxCurrent(address, int(config['MOTOR_CONFIG']['max_corner_current']*100))
				#self.rc.SetM2MaxCurrent(address, int(config['MOTOR_CONFIG']['max_corner_current']*100))

				self.enc_min.append(self.rc.ReadM1PositionPID(address)[-2])
				self.enc_min.append(self.rc.ReadM2PositionPID(address)[-2])
				self.enc_max.append(self.rc.ReadM1PositionPID(address)[-1])
				self.enc_max.append(self.rc.ReadM2PositionPID(address)[-1])

			else:
				#self.rc.SetM1MaxCurrent(address, int(config['MOTOR_CONFIG']['max_drive_current']*100))
				#self.rc.SetM2MaxCurrent(address, int(config['MOTOR_CONFIG']['max_drive_current']*100))
				self.rc.ResetEncoders(address)

		rospy.set_param('enc_min', str(self.enc_min)[1:-1])
		rospy.set_param('enc_max', str(self.enc_max)[1:-1])

	def cornerToPosition(self,tick):
		"""
		Send position commands to the corner motor

		:param list tick: A list of ticks for each of the corner motors to
		move to, if tick[i] is 0 it instead stops that motor from moving
		"""
		# These values could potentially need tuning still
		speed = 1000
		accel = 2000
		for i in range(4):
                        # roboclaw index: 3 or 4 --> roboclaw 4 or 5 (starts at 0)
			index = int(math.ceil((i+1)/2.0)+2)
			
			if tick[i] != -1:
				if (i % 2):
					self.rc.SpeedAccelDeccelPositionM2(self.address[index],accel,speed,accel,tick[i],1)
				else:
					self.rc.SpeedAccelDeccelPositionM1(self.address[index],accel,speed,accel,tick[i],1)

	def sendMotorDuty(self, motorID, speed):
		"""
		Wrapper method for an easier interface to control the drive motors,

		sends open-loop commands to the motors

		:param int motorID: number that corresponds to each physical motor
		:param int speed: Speed for each motor, range from 0-127

		"""
		#speed = speed/100.0
		#speed *= 0.5
		addr = self.address[int(motorID/2)]
		if speed > 0:
			if not motorID % 2: command = self.rc.ForwardM1
			else:               command = self.rc.ForwardM2
		else:
			if not motorID % 2: command = self.rc.BackwardM1
			else:               command = self.rc.BackwardM2

		speed = abs(int(speed * 127))

		return command(addr,speed)

	def sendSignedDutyAccel(self,motorID,speed):
		addr = self.address[int(motorID/2)]

		if speed >0: accel = self.accel_pos
		else: accel = self.accel_neg

		if not motorID % 2: command = self.rc.DutyAccelM1
		else:               command = self.rc.DutyAccelM2

		speed = int(32767 * speed/100.0)
		return command(addr,accel,speed)

	def getCornerEnc(self):
		enc = []
		for i in range(4):
			index = int(math.ceil((i+1)/2.0)+2)
			if not (i % 2):
				enc.append(self.rc.ReadEncM1(self.address[index])[1])
			else:
				enc.append(self.rc.ReadEncM2(self.address[index])[1])
		self.enc = enc
		return enc


	@staticmethod
	def tick2deg(tick,e_min,e_max):
		"""
		Converts a tick to physical degrees

		:param int tick : Current encoder tick
		:param int e_min: The minimum encoder value based on physical stop
		:param int e_max: The maximum encoder value based on physical stop
		"""
		return (tick - (e_max + e_min)/2.0) * (90.0/(e_max - e_min))

	def getCornerEncAngle(self):
		if self.enc[0] == None:
			return -1
		deg = [None] *4

		for i in range(4):
			deg[i] = int(self.tick2deg(self.enc[i],self.enc_min[i],self.enc_max[i]))

		return deg

	def getDriveEnc(self):
		enc = [None]*6
		for i in range(6):
			if not (i % 2):
				enc[i] = self.rc.ReadEncM1(self.address[int(math.ceil(i/2))])[1]
			else:
				enc[i] = self.rc.ReadEncM2(self.address[int(math.ceil(i/2))])[1]
		return enc

	def getBattery(self):
		return self.rc.ReadMainBatteryVoltage(self.address[0])[1]
		
	def getTemp(self):
		temp = [None] * 5
		for i in range(5):
			temp[i] = self.rc.ReadTemp(self.address[i])[1]
		return temp
	
	def getCurrents(self):
		currents = [None] * 10
		for i in range(5):
			currs = self.rc.ReadCurrents(self.address[i])
			currents[2*i] = currs[1]
			currents[(2*i) + 1] = currs[2]
		return currents

	def getErrors(self):
		return self.err

	def killMotors(self):
		"""Stops all motors on Rover"""
		for i in range(5):
			self.rc.ForwardM1(self.address[i],0)
			self.rc.ForwardM2(self.address[i],0)

	def errorCheck(self):
		"""Checks error status of each motor controller, returns 0 if any errors occur"""

		for i in range(len(self.address)):
			self.err[i] = self.rc.ReadError(self.address[i])[1]
		for error in self.err:
			if error:
				self.killMotors()
				#self.writeError()
				rospy.loginfo("Motor controller Error", error)
		return 1

	def writeError(self):
		"""Writes the list of errors to a text file for later examination"""

		f = open('errorLog.txt','a')
		errors = ','.join(str(e) for e in self.err)
		f.write('\n' + 'Errors: ' + '[' + errors + ']' + ' at: ' + str(datetime.datetime.now()))
		f.close()


if __name__ == "__main__":
	rospy.init_node("roboclaw wrapper", log_level=rospy.DEBUG)
	rospy.loginfo("Starting the roboclaw wrapper node")

	wrapper = RoboclawWrapper()
	rospy.on_shutdown(wrapper.killMotors)
	wrapper.run()
