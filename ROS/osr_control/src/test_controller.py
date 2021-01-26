#!/usr/bin/env python

"""
Test velocity controller. For developer use only.
"""

import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Float64


if __name__ == "__main__":

    rospy.init_node("test_control", log_level=rospy.DEBUG)

    pub = rospy.Publisher('/control/osr_controllers/drive_velocity_controller/command', Float64MultiArray, queue_size=2)
    rospy.sleep(0.5)

    cmd = Float64MultiArray()
    cmd.layout.dim.append(MultiArrayDimension(label="height", size=10, stride=10))
    cmd.data = 10 * [0.0]
    cmd.data[:6] = 6 * [100.0]
    cmd_zero = Float64MultiArray()
    cmd_zero.layout.dim.append(MultiArrayDimension(label="height", size=10, stride=10))
    cmd_zero.data = 10 * [0.0]

    r = rospy.Rate(5)
    time_now = rospy.Time.now()
    while rospy.Time.now() < time_now + rospy.Duration(5):
        pub.publish(cmd)
        r.sleep()
    
    pub.publish(cmd_zero)
    
    rospy.logdebug("Done.")
