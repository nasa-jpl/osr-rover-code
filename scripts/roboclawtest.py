from time import sleep
import sys
from os import path
# need to add the roboclaw.py file in the path
sys.path.append(path.join(path.expanduser('~'), 'osr_ws/src/osr-rover-code/ROS/osr/src'))
from roboclaw import Roboclaw

if __name__ == "__main__":
    
    address = int(sys.argv[1]) 
    roboclaw = Roboclaw("/dev/ttyS0", 115200)
    roboclaw.Open()

    print roboclaw.ReadVersion(address)
    print roboclaw.ReadEncM1(address)
