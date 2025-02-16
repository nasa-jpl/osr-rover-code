# A short and sweet script to test communication with a single roboclaw motor controller.
# usage 
#   $ python roboclawtest.py 128
# Things are working if you don't get an error and you see something like:
# (1, 'USB Roboclaw 2x7a v4.1.34\n')
# (1, 4314, 128)

from time import sleep
import sys
from os import path
# need to add the roboclaw.py file in the path
sys.path.append(path.join(path.expanduser('~'), 'osr_ws/src/osr-rover-code/ROS/osr_control/osr_control'))
from roboclaw import Roboclaw

if __name__ == "__main__":
    
    address = int(sys.argv[1]) 
    roboclaw0 = Roboclaw("/dev/serial0", 115200)
    roboclaw1 = Roboclaw("/dev/serial1", 115200)
    connected0 = roboclaw0.Open() == 1
    connected1 = roboclaw1.Open() == 1
    one_connected = False
    if connected0:
        print("Connected to /dev/serial0.")
        print(f"Address: {address}")
        print("ReadVersion:", roboclaw0.ReadVersion(address))
        print("ReadEncM1:", roboclaw0.ReadEncM1(address))
        battery = roboclaw0.ReadMainBatteryVoltage(address)
        print(f"Address {address} - ReadMainBatteryVoltage: {battery}")
        one_connected = True
    if connected1:
        print("Connected to /dev/serial1.")
        print(f"Address: {address}")
        print("ReadVersion:", roboclaw1.ReadVersion(address))
        print("ReadEncM1:", roboclaw1.ReadEncM1(address))
        battery = roboclaw1.ReadMainBatteryVoltage(address)
        print(f"Address {address} - ReadMainBatteryVoltage: {battery}")
        one_connected = True
    if not one_connected:
        print("Could not open comport /dev/serial0 or /dev/serial1, make sure it has the correct permissions and is available")
