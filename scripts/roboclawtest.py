# A short and sweet script to test communication with a single roboclaw motor controller.
# usage 
#   $ python roboclawtest.py 128

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

    for rc in [roboclaw0, roboclaw1]:
        connected = rc.Open() == 1
        if not connected:
            continue
        print(f"Found open comport {rc._port}, attempting to communicate with address {address}")
        version = rc.ReadVersion(address)
        if version[0] == 1:
            print(f"Successfully connected, roboclaw firmware version: {version[1]}")
        else:
            print("Failed to read version, did not get response from roboclaw. Make sure it is properly connected.")
            continue
        enc1 = rc.ReadEncM1(address)
        enc2 = rc.ReadEncM2(address)
        battery = rc.ReadMainBatteryVoltage(address)
        if enc1[0] == 1 and enc2[0] == 1 and battery[0] == 1:
            print(f"Encoder for M1 = {enc1[1]}, M2 = {enc2[1]}. Roboclaw input voltage: {battery[1]}")
            break  # don't need to try the other port
        else:
            print("Failed to read encoder and/or battery values, connection is unstable. Check your wiring.")
            print(f"Raw responses: Encoder M1: {enc1}, Encoder M2: {enc2}, Battery: {battery}. A (0, value) response means no response.")

    if not connected:
        print("Could not open comport /dev/serial0 or /dev/serial1, make sure it has the correct permissions and is available")
