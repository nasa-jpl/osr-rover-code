# A short and sweet script to test movement of a single roboclaw motor channel
#   $ python roboclaw_movemotor.py 130

from time import sleep
import sys
from os import path
# need to add the roboclaw.py file in the path
sys.path.append(path.join(path.expanduser('~'), 'osr_ws/src/osr-rover-code/ROS/osr_control/osr_control'))
from roboclaw import Roboclaw

BAUD_RATE = 115200

def test_connection(address):
    roboclaw0 = Roboclaw("/dev/serial0", BAUD_RATE)
    roboclaw1 = Roboclaw("/dev/serial1", BAUD_RATE)
    connected0 = roboclaw0.Open() == 1
    connected1 = roboclaw1.Open() == 1
    if connected0:
        print("Connected to /dev/serial0.")
        print(f"version: {roboclaw0.ReadVersion(address)}")
        print(f"encoders: {roboclaw0.ReadEncM1(address)}")
        return roboclaw0
    elif connected1:
        print("Connected to /dev/serial1.")
        print(f"version: {roboclaw1.ReadVersion(address)}")
        print(f"encoders: {roboclaw1.ReadEncM1(address)}")
        return roboclaw1
    else:
        print("Could not open comport /dev/serial0 or /dev/serial1, make sure it has the correct permissions and is available")
        return None
    

if __name__ == "__main__":
    
    address = int(sys.argv[1]) 
    
    rc = test_connection(address)

    ## Set accel
    accel_max = 2**15-1
    accel_rate = 0.5
    drive_accel = int(accel_max * accel_rate)
    # drive_accel = 16383 # good accel, speedup not really noticeable
    drive_accel = 100 # rul slow accel

    ## Set speed
    roboclaw_overflow = 2**15-1   # 32767 max speed (way too fast)
    # target_qpps = roboclaw_overflow  # very fast, max speed
    # target_qpps = 2048  # fairly fast
    # target_qpps = 1000  # pretty good test speed
    # target_qpps = -1000
    target_qpps = 100  # pretty slow

    # target_qpps = max(-roboclaw_overflow, min(roboclaw_overflow, target_qpps))
    
    # rc.SpeedAccelM1(address, drive_accel, 1000)
    # sleep(1)
    # rc.ForwardM1(address, 0)
    # sleep(0.5)
    # rc.SpeedAccelM1(address, drive_accel, -1000)
    # sleep(1)
    # rc.ForwardM1(address, 0)

    # rc.SpeedAccelM2(address, drive_accel, 1000)
    # sleep(1)
    # rc.ForwardM2(address, 0)
    # sleep(0.5)
    # rc.SpeedAccelM2(address, drive_accel, -1000)
    # sleep(1)
    # rc.ForwardM2(address, 0)

    print("M1:")
    # Move M1
    for speed_cmd in range(10, target_qpps, 10):
        rc.SpeedAccelM1(address, drive_accel, speed_cmd)
        speed = rc.ReadSpeedM1(address)
        print(f'speed: {speed[1]}')
        sleep(0.5)
    sleep(1)
    # stop motor
    rc.ForwardM1(address, 0)

    print("M2:")
    # Move M2
    for speed_cmd in range(10, target_qpps, 10):
        rc.SpeedAccelM2(address, drive_accel, speed_cmd)
        speed = rc.ReadSpeedM2(address)
        print(f'speed: {speed[1]}')
        sleep(0.5)
    sleep(1)
    # stop motor
    rc.ForwardM2(address, 0)
