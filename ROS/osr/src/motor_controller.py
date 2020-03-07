#!/usr/bin/env python
import time
import rospy
from osr_msgs.msg import Commands, Encoder, Status
from roboclaw_wrapper import RoboclawWrapper

global mutex
mutex = False
roboclaw_wrapper = RoboclawWrapper()

def callback(cmds):
	global mutex	
	rospy.loginfo(cmds)
	while mutex:
		time.sleep(0.001)
		#print "cmds are being buffered"
	mutex = True
	# PUT THIS BACK IN
	roboclaw_wrapper.cornerToPosition(cmds.corner_motor)
	for i in range(6):
		# PUT THIS BACK IN
		#roboclaw_wrapper.sendMotorDuty(i,cmds.drive_motor[i])
		roboclaw_wrapper.sendSignedDutyAccel(i,cmds.drive_motor[i])
		pass
	mutex = False

def shutdown():
	print "killing motors"
	roboclaw_wrapper.killMotors()


if __name__ == "__main__":

	rospy.init_node("motor_controller")
	rospy.loginfo("Starting the motor_controller node")
	rospy.on_shutdown(shutdown)
	sub = rospy.Subscriber("/robot_commands",Commands,callback)
	enc_pub = rospy.Publisher("/encoder", Encoder, queue_size =1)
	status_pub = rospy.Publisher("/status", Status, queue_size =1)

	rate = rospy.Rate(5)

	status = Status()
	enc   = Encoder()
	
	enc.abs_enc      		=[1000]*4
	enc.abs_enc_angles 		=[-100]*4
	status.battery          = 0
	status.temp         	=[0]*5
	status.current      	=[0]*10
	status.error_status 	=[0]*5
	
	counter = 0
	while not rospy.is_shutdown():

		while mutex:
			time.sleep(0.001)
		mutex = True
		enc.abs_enc = roboclaw_wrapper.getCornerEnc()
		#mc_data.abs_enc_angles = roboclaw_wrapper.getCornerEncAngle()
		if (counter >= 10):
			
			status.battery = roboclaw_wrapper.getBattery()
			status.temp = roboclaw_wrapper.getTemp()
			status.current = roboclaw_wrapper.getCurrents()
			status.error_status = roboclaw_wrapper.getErrors()
			status_pub.publish(status)
			counter = 0
			
		mutex = False
		enc_pub.publish(enc)
		counter += 1
		rate.sleep()
	rospy.spin()
