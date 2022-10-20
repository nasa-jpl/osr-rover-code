"""
Simple test script for PCA9685 PWM motor control

Moves two motors back and forth

Setup:
$ pip3 install adafruit-circuitpython-servokit

Instructions: https://learn.adafruit.com/adafruit-16-channel-servo-driver-with-raspberry-pi/using-the-adafruit-library
"""
from time import sleep

# probably installed at /home/ubuntu/.local/lib/python3.8/site-packages/adafruit_servokit.py
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

# kit.servo[0].actuation_range = 160

while True:
    # pwm channel 0
    kit.servo[0].angle = 180
    # pwm channel 2
    kit.servo[2].angle = 0
    sleep(1)
    kit.servo[0].angle = 0
    kit.servo[2].angle = 180
    sleep(1)
