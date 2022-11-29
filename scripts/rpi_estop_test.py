"""
Test script to run a motor and then estop it in the middle

first need to `sudo apt-get install python3-rpi.gpio` on the rpi

Uses both gpio and motor commands, see rpi_gpio_test.py and roboclaw_movemotor.py for more info

Hardcoded to use roboclaw address 130 and channel M2
"""

from time import sleep
import sys
from os import path

import RPi.GPIO as gpio

# need to add the roboclaw.py file in the path
sys.path.append(path.join(path.expanduser('~'), 'osr_ws/src/osr-rover-code/ROS/osr_control/osr_control'))
from roboclaw import Roboclaw


## GPIO and ESTOP setup

# on pcb rev 2.0, this is the gpio pin connected to E_STOP_pi
gpio_num = 23

gpio.setwarnings(False)
# there are two modes, BCM and BOARD
#  requried reading: https://raspberrypi.stackexchange.com/a/12967
#  basically, we choose BCM here so that we can choose the pin number based on 
#  the GPIOXX labels on the pins in the rpi datasheet
gpio.setmode(gpio.BCM)
gpio.setup(gpio_num, gpio.OUT)

# enable motors
gpio.output(gpio_num, True)



## Move motor and estop it

address = 130

BAUD_RATE = 115200
rc = Roboclaw('/dev/serial1', BAUD_RATE)

rc.Open()
version_response = rc.ReadVersion(address)
print(f'version_response: {version_response}')

# accel and target speed (see roboclaw_movemotor.py for more info)
drive_accel = 16383 # good accel, speedup not really noticeable
target_qpps = 1000  # pretty good test speed


# NOTE: assumes channel M2

print('Move motor for 3 seconds')
rc.SpeedAccelM2(address, drive_accel, target_qpps)

sleep(3)

print('Estop motor!')
gpio.output(gpio_num, False)

sleep(3)

# # stop motor
# rc.ForwardM2(address, 0)

# will unassert pin again (which also effectively estops)
print('Exiting')
gpio.cleanup()