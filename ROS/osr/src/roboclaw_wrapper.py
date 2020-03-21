#!/usr/bin/env python

import time
import serial
import math
import rospy

from roboclaw import Roboclaw

from sensor_msgs.msg import JointState
from osr_msgs.msg import CommandDrive, CommandCorner, Status


class RoboclawWrapper(object):
	"""Interface between the roboclaw motor drivers and the higher level rover code"""

	def __init__(self):
		rospy.loginfo( "Initializing motor controllers")

		# initialize attributes
		self.rc = None
		self.err = [None] * 5
		self.address = []
		self.current_enc_vals = None
		self.mutex = False

		self.establish_roboclaw_connections()
		self.killMotors()  # don't move at start
		self.setup_encoders()

		self.roboclaw_mapping = rospy.get_param('/roboclaw_mapping')
		self.encoder_limits = {}

		# save settings to non-volatile (permanent) memory
		for address in self.address:
			self.rc.WriteNVM(address)

		for address in self.address:
			self.rc.ReadNVM(address)

		accel_max = 655359
		accel_rate = 0.5
		self.accel_pos = int((accel_max /2) + accel_max * accel_rate)
		self.accel_neg = int((accel_max /2) - accel_max * accel_rate)
		self.errorCheck()

		time.sleep(2)
		self.killMotors()

		# set up publishers and subscribers
		self.cmd_sub = rospy.Subscriber("/cmd_corner", CommandCorner, self.corner_cmd_cb)
		self.cmd_sub = rospy.Subscriber("/cmd_drive", CommandDrive, self.drive_cmd_cb)
		self.enc_pub = rospy.Publisher("/encoder", JointState, queue_size=1)
		self.status_pub = rospy.Publisher("/status", Status, queue_size=1)

	def run(self):
		"""Blocking loop which runs after initialization has completed"""
		rate = rospy.Rate(5)

		status = Status()

		counter = 0
		while not rospy.is_shutdown():

			while self.mutex and not rospy.is_shutdown():
				rate.sleep()
			self.mutex = True

			# read from roboclaws and publish
			self.read_encoder_values()
			self.enc_pub.publish(self.current_enc_vals)

			if (counter >= 10):
				status.battery = self.getBattery()
				status.temp = self.getTemp()
				status.current = self.getCurrents()
				status.error_status = self.getErrors()
				self.status_pub.publish(status)
				counter = 0

			self.mutex = False
			counter += 1
			rate.sleep()

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
		"""Set up the encoders"""
		for motor_name, properties in self.roboclaw_mapping:
			if "corner" in motor_name:
				enc_min, enc_max = self.read_encoder_limits(properties["address"], properties["channel"])
				self.encoder_limits["motor_name"] = (enc_min, enc_max)
			else:
				self.rc.ResetEncoders(properties["address"])

	def read_encoder_values(self):
		"""Query roboclaws and update current motors status in encoder ticks"""
		enc_msg = JointState()
		enc_msg.header.stamp = rospy.Time.now()
		for motor_name, properties in self.roboclaw_mapping:
			enc_msg.name.append(motor_name)
			position = self.read_encoder_position(properties["address"], properties["channel"])
			velocity = self.read_encoder_velocity(properties["address"], properties["channel"])
			current = self.read_encoder_current(properties["address"], properties["channel"])
			enc_msg.position.append(self.tick2position(position,
													   properties['enc_min'],
													   properties['enc_max'],
													   properties['ticks_per_rev']))
			enc_msg.velocity.append(self.qpps2velocity(velocity,
													   properties['ticks_per_rev'],
													   properties['gear_ratio']))
			enc_msg.effort.append(current)

		self.current_enc_vals = enc_msg

	def corner_cmd_cb(self, cmd):
		r = rospy.Rate(10)
		rospy.logdebug("Corner command callback received: {}".format(cmd))

		while self.mutex and not rospy.is_shutdown():
			r.sleep()

		self.mutex = True

		# convert position to tick
		encmin, encmax = self.encoder_limits["corner_left_front"]
		left_front_tick = self.position2tick(cmd.left_front_pos, encmin, encmax,
											 self.roboclaw_mapping["corner_left_front"]["ticks_per_rev"])
		encmin, encmax = self.encoder_limits["corner_left_back"]
		left_back_tick = self.position2tick(cmd.left_back_pos, encmin, encmax,
											self.roboclaw_mapping["corner_left_back"]["ticks_per_rev"])
		encmin, encmax = self.encoder_limits["corner_left_back"]
		right_back_tick = self.position2tick(cmd.right_back_pos, encmin, encmax,
											 self.roboclaw_mapping["corner_right_back"]["ticks_per_rev"])
		encmin, encmax = self.encoder_limits["corner_left_back"]
		right_front_tick = self.position2tick(cmd.right_front_pos, encmin, encmax,
											  self.roboclaw_mapping["corner_right_front"]["ticks_per_rev"])

		self.send_position_cmd(self.roboclaw_mapping["corner_left_front"]["address"],
							   self.roboclaw_mapping["corner_left_front"]["channel"],
							   left_front_tick)
		self.send_position_cmd(self.roboclaw_mapping["corner_left_back"]["address"],
							   self.roboclaw_mapping["corner_left_back"]["channel"],
							   left_back_tick)
		self.send_position_cmd(self.roboclaw_mapping["corner_right_back"]["address"],
							   self.roboclaw_mapping["corner_right_back"]["channel"],
							   right_back_tick)
		self.send_position_cmd(self.roboclaw_mapping["corner_right_front"]["address"],
							   self.roboclaw_mapping["corner_right_front"]["channel"],
							   right_front_tick)
		self.mutex = False

	def drive_cmd_cb(self, cmd):
		r = rospy.Rate(10)
		rospy.logdebug("Drive command callback received: {}".format(cmd))

		while self.mutex and not rospy.is_shutdown():
			r.sleep()

		props = self.roboclaw_mapping["drive_left_front"]
		vel_cmd = self.velocity2qpps(cmd.left_front_vel, props["ticks_per_rev"], props["gear_ratio"])
		self.send_velocity_cmd(props["address"], props["channel"], vel_cmd)

		props = self.roboclaw_mapping["drive_left_middle"]
		vel_cmd = self.velocity2qpps(cmd.left_middle_vel, props["ticks_per_rev"], props["gear_ratio"])
		self.send_velocity_cmd(props["address"], props["channel"], vel_cmd)

		props = self.roboclaw_mapping["drive_left_back"]
		vel_cmd = self.velocity2qpps(cmd.left_back_vel, props["ticks_per_rev"], props["gear_ratio"])
		self.send_velocity_cmd(props["address"], props["channel"], vel_cmd)

		props = self.roboclaw_mapping["drive_right_back"]
		vel_cmd = self.velocity2qpps(cmd.right_back_vel, props["ticks_per_rev"], props["gear_ratio"])
		self.send_velocity_cmd(props["address"], props["channel"], vel_cmd)

		props = self.roboclaw_mapping["drive_right_middle"]
		vel_cmd = self.velocity2qpps(cmd.right_middle_vel, props["ticks_per_rev"], props["gear_ratio"])
		self.send_velocity_cmd(props["address"], props["channel"], vel_cmd)

		props = self.roboclaw_mapping["drive_right_front"]
		vel_cmd = self.velocity2qpps(cmd.right_front_vel, props["ticks_per_rev"], props["gear_ratio"])
		self.send_velocity_cmd(props["address"], props["channel"], vel_cmd)

	def send_position_cmd(self, address, channel, target_tick):
		"""
		Wrapper around one of the send position commands

		:param address:
		:param channel:
		:param target_tick:
		"""
		cmd_args = [self.default_accel, self.corner_max_vel, self.default_accel, target_tick, 1]
		if channel == "M1":
			return self.rc.SpeedAccelDeccelPositionM1(address, *cmd_args)
		elif channel == "M2":
			return self.rc.SpeedAccelDeccelPositionM2(address, *cmd_args)
		else:
			raise AttributeError("Received unknown channel '{}'. Expected M1 or M2".format(channel))

	def read_encoder_position(self, address, channel):
		"""Wrapper around self.rc.ReadEncM1 and self.rcReadEncM2 to simplify code"""
		if channel == "M1":
			return self.rc.ReadEncM1(address)
		elif channel == "M2":
			return self.rc.ReadEncM2(address)
		else:
			raise AttributeError("Received unknown channel '{}'. Expected M1 or M2".format(channel))

	def read_encoder_limits(self, address, channel):
		"""Wrapper around self.rc.ReadPositionPID and returns subset of the data

		:return: (enc_min, enc_max)
		"""
		if channel == "M1":
			result = self.rc.ReadM1PositionPID(address)
		elif channel == "M2":
			result = self.rc.ReadM2PositionPID(address)
		else:
			raise AttributeError("Received unknown channel '{}'. Expected M1 or M2".format(channel))

		return (result[-2], result[-1])

	def send_velocity_cmd(self, address, channel, target_qpps):
		"""
		Wrapper around one of the send velocity commands

		:param address:
		:param channel:
		:param target_qpps:
		"""
		accel = self.accel_pos
		if target_qpps < 0:
			accel = self.accel_neg
		if channel == "M1":
			return self.rc.DutyAccelM1(address, accel, target_qpps)
		elif channel == "M2":
			return self.rc.DutyAccelM2(address, accel, target_qpps)
		else:
			raise AttributeError("Received unknown channel '{}'. Expected M1 or M2".format(channel))

	def read_encoder_velocity(self, address, channel):
		"""Wrapper around self.rc.ReadSpeedM1 and self.rcReadSpeedM2 to simplify code"""
		if channel == "M1":
			return self.rc.ReadSpeedM1(address)
		elif channel == "M2":
			return self.rc.ReadSpeedM2(address)
		else:
			raise AttributeError("Received unknown channel '{}'. Expected M1 or M2".format(channel))

	def read_encoder_current(self, address, channel):
		"""Wrapper around self.rc.ReadCurrents to simplify code"""
		if channel == "M1":
			return self.rc.ReadCurrents(address)[0]
		elif channel == "M2":
			return self.rc.ReadCurrents(address)[1]
		else:
			raise AttributeError("Received unknown channel '{}'. Expected M1 or M2".format(channel))

	def tick2position(self, tick, enc_min, enc_max, ticks_per_rev):
		"""
		Convert the absolute position from ticks to radian relative to the middle position

		:param tick:
		:param enc_min:
		:param enc_max:
		:param ticks_per_rev:
		:return:
		"""
		ticks_per_rad = ticks_per_rev / (2 * math.pi)
		mid = enc_min + (enc_max - enc_min) / 2
		return (tick - mid) / ticks_per_rad

	def position2tick(self, position, enc_min, enc_max, ticks_per_rev):
		"""
		Convert the absolute position from radian relative to the middle position to ticks

		:param position:
		:param enc_min:
		:param enc_max:
		:param ticks_per_rev:
		:return:
		"""
		ticks_per_rad = ticks_per_rev / (2 * math.pi)
		mid = enc_min + (enc_max - enc_min) / 2
		return mid + position * ticks_per_rad

	def qpps2velocity(self, qpps, ticks_per_rev, gear_ratio):
		"""
		Convert the given quadrature pulses per second to radian/s

		:param qpps:
		:param ticks_per_rev:
		:param gear_ratio:
		:return:
		"""
		return qpps / (2 * math.pi * gear_ratio * ticks_per_rev)

	def velocity2qpps(self, velocity, ticks_per_rev, gear_ratio):
		"""
		Convert the given velocity to quadrature pulses per second

		:param velocity: rad/s
		:param ticks_per_rev:
		:param gear_ratio:
		:return:
		"""
		return velocity * 2 * math.pi * gear_ratio * ticks_per_rev

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
				rospy.logerr("Motor controller Error: \n'{}'".format(error))
		return 1


if __name__ == "__main__":
	rospy.init_node("roboclaw wrapper", log_level=rospy.DEBUG)
	rospy.loginfo("Starting the roboclaw wrapper node")

	wrapper = RoboclawWrapper()
	rospy.on_shutdown(wrapper.killMotors)
	wrapper.run()
