#!/usr/bin/env python
import time
import rospy
from osr_msgs.msg import Status
from screen import LedScreen

screen = LedScreen()

def callback(data):
	#rospy.loginfo(data)
	#send usb-> ttl serial commands
	screen.build_msg(1,data.battery,data.error_status,data.temp,data.current)
	screen.check_for_afffirm()

def shutdown():
	screen.transistion_to_idle()
	return 0

if __name__ == "__main__":

	rospy.init_node("led_screen")
	rospy.loginfo("Starting the led_screen node")
	rospy.on_shutdown(shutdown)
	sub = rospy.Subscriber("/status", Status, callback)
	rospy.spin()
