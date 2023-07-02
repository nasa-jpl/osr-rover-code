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

sleep(0.1)

# for i in range(4):
#     kit.servo[i].actuation_range = 300
#     kit.servo[i].set_pulse_width_range(500, 2500)
#     kit.servo[i].angle = 150
#     sleep(0.5)

i=3
kit.servo[i].actuation_range = 300
kit.servo[i].set_pulse_width_range(500, 2500)
kit.servo[i].angle = 160


# while True:
#     # pwm channel 0
#     for i in [0, 100]:
#         for ind in range(1):
#             # print(kit.servo[ind].actuation_range)
#             srv = kit.servo[ind]
#             srv.angle = i
#             for t in range(10):
#                 print(srv.angle)
#                 sleep(0.01)
#         sleep(1)


    # kit.servo[0].angle = 150
    # kit.servo[1].angle = 100
    # for i in [100, 150]:
    #     kit.servo[0].angle = i
    #     kit.servo[1].angle = i
    #     kit.servo[2].angle = i
    #     kit.servo[3].angle = i
    # # pwm channel 2
    # kit.servo[2].angle = 0
    # sleep(1)
    # kit.servo[0].angle = 0
    # kit.servo[2].angle = 180
        # sleep(1)
