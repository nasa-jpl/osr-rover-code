#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64MultiArray
import math

global mode
global last
global counter
mode,counter = 0,0
last = time.time()

def callback(data):
    global mode
    global counter
    global last

    led_msg = Int64MultiArray()

    y = data.axes[1]
    x = data.axes[0]
    x1 = data.axes[3]
    rt = data.axes[2]

    dpad = data.buttons[11:]
    if 1 in dpad: mode = dpad.index(1)
    now = time.time()

    led_msg.data = [mode,1]
    if now - last > 0.75:
        counter +=1
    else:
        counter = 0
    if counter > 3:
        led_msg.data = [mode,0]

    last = time.time()
    led_pub.publish(led_msg)
    #cmd = cartesian2polar_45(x,y)
    cmd = two_joy(x1,y,rt)
    joy_out = Twist()
    joy_out.linear.x = cmd[0]
    joy_out.angular.z = cmd[1]
    pub.publish(joy_out)

def old(x,y):
    if y < 0: direction = -1
    else: direction = 1

    r = int(100*math.sqrt(x*x + y*y)) * direction

    if r > 100: r = 100
    elif r < -100: r = -100

    if -15 <= r <= 15:
        r = 0
        theta = 0
    #there is some small issue where every once and a while the steering
    #goes to max negative for one or two values at very small values of y
    #this eventually needs to be fixed
    elif -0.01 <= y <= 0.01:
        theta = 100 * direction
    else:
        try:
            theta = int(math.degrees(math.atan(x/y)) * direction * (10/9.0))
        except:
            theta = 0
    return r,theta

def cartesian2polar_45(x,y):
    if y < 0: direction = -1
    else: direction = 1

    r = math.sqrt(x*x + y*y)
    r = min(r,1.0)
    try:
        theta = math.degrees(math.atan2(x,y))
    except:
        if x >0: theta = 90
        else: theta = -90
    if (-45 <= theta <= 45) or (135 <=theta <= 180) or (-180 <=theta <= -135):
        vel = int(r * 100) * direction
    else:
        vel = (2.0/math.sqrt(2))*y*100
    vel = min(100,vel)
    vel = max(-100,vel)

    '''
    if y >= 0:
        steering = theta * 10/9.0
    else:
        if x >= 0: x_dir = 1
        else: x_dir = -1
        steering = (180 - abs(theta)) * (10/9.0) * x_dir
    '''
    steering = x * 2.0/math.sqrt(2) * 100
    steering = min(100,steering)
    steering = max(-100,steering)
    return (int(vel),int(steering))

def two_joy(x,y,rt):
    max_vel = 0.3
    forward_vel = y * max_vel
    rotation = x * math.pi / 4

    return forward_vel, rotation

if __name__ == '__main__':
    global pub
    #global led_pub
    rospy.init_node('joystick')
    rospy.loginfo('joystick started')

    sub = rospy.Subscriber("/joy", Joy, callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    led_pub = rospy.Publisher('led_cmds', Int64MultiArray, queue_size=1)

    rospy.spin()
