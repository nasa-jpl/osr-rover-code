#!/usr/bin/env python

import rospy
import math

from osr_msgs.msg import Joystick, DriveCommand, CornerCommand


class Rover(object):
	"""Math and motor control algorithms to move the rover"""

	def __init__(self):
		distances = rospy.get_param('mech_dist','7.254,10.5,10.5,10.073').split(",")
		self.d1 = float(distances[0])
		self.d2 = float(distances[1])
		self.d3 = float(distances[2])
		self.d4 = float(distances[3])

		rospy.Subscriber("/joystick", Joystick, self.cmd_cb)

		self.corner_cmd_pub = rospy.Publisher("/cmd_corner", CornerCommand, queue_size=1)
		self.drive_cmd_pub = rospy.Publisher("/cmd_drive", DriveCommand, queue_size=1)

	def cmd_cb(self, msg):
		corner_cmd_msg = self.calculate_corner_positions(msg.steering)
		drive_cmd_msg = self.calculate_drive_velocities(msg.vel, msg.steering)
		self.corner_cmd_pub.publish(corner_cmd_msg)
		self.drive_cmd_pub.publish(drive_cmd_msg)

	def calculate_drive_velocities(self, speed, current_radius):
		"""
		Calculate target velocities for the drive motors based on desired speed and current turning radius

		:param int speed: Drive speed command range from -100 to 100
		:param int radius: Current turning radius range from -250 to 250
		"""
		cmd_msg = DriveCommand()
		if (speed == 0):
			return cmd_msg

		if (abs(current_radius) <= 5):  # No turning radius, all wheels same speed
			cmd_msg.left_front = speed
			cmd_msg.left_middle = speed
			cmd_msg.left_back = speed
			cmd_msg.right_back = speed
			cmd_msg.right_middle = speed
			cmd_msg.right_front = speed
		else:
			radius = 250 - (230 * abs(current_radius)) / 100.0
			if radius < 0:
				rmax *= -1
			rmax = radius + self.d4

			a = math.pow(self.d2, 2)
			b = math.pow(self.d3, 2)
			c = math.pow(abs(radius) + self.d1, 2)
			d = math.pow(abs(radius) - self.d1, 2)
			e = abs(radius) - self.d4
			rmax_float = float(rmax)

			v1 = int(speed * (math.sqrt(b + d)) / rmax_float)
			v2 = int(speed * e / rmax_float)  # Slowest wheel
			v3 = int((speed * math.sqrt(a + d)) / rmax_float)
			v4 = int((speed * math.sqrt(a + c)) / rmax_float)
			v5 = int(speed)  # Fastest wheel
			v6 = int((speed * math.sqrt(b + c)) / rmax_float)

			if current_radius < 0:
				cmd_msg.left_front = v4
				cmd_msg.left_middle = v5
				cmd_msg.left_back = v6
				cmd_msg.right_back = v1
				cmd_msg.right_middle = v2
				cmd_msg.right_front = v3
			else:
				cmd_msg.left_front = v3
				cmd_msg.left_middle = v2
				cmd_msg.left_back = v1
				cmd_msg.right_back = v6
				cmd_msg.right_middle = v5
				cmd_msg.right_front = v4

			return cmd_msg

	def calculate_corner_positions(self, radius):
		"""
		Takes a turning radius and calculates what angle [rad] each corner should be at

		:param int radius: Radius drive command, ranges from -100 (turning left) to 100 (turning right)
		"""
		#Scaled from 250 to 20 inches. For more information on these numbers look at the Software Controls.pdf
		cmd_msg = CornerCommand()

		if radius == 0:
			r = 250
		elif -100 <= radius <= 100 and radius != 0:
			r = 220 - abs(radius) * 250 / 100
		else:
			r = 250

		if r == 250:
			return cmd_msg

		ang1 = math.atan(self.d1 / (abs(r) + self.d3))
		ang2 = math.atan(self.d2 / (abs(r) + self.d3))
		ang3 = math.atan(self.d2 / (abs(r) - self.d3))
		ang4 = math.atan(self.d1 / (abs(r) - self.d3))

		if radius > 0:
			cmd_msg.left_front = ang4
			cmd_msg.left_back = -ang3
			cmd_msg.right_back = -ang2
			cmd_msg.right_front = ang1
		else:
			cmd_msg.left_front = -ang2
			cmd_msg.left_back = ang1
			cmd_msg.right_back = ang4
			cmd_msg.right_front = -ang3

		return cmd_msg


if __name__ == '__main__':
	rospy.init_node('rover', log_level=rospy.DEBUG)
	rospy.loginfo("Starting the rover node")
	rover = Rover()
	rospy.spin()
