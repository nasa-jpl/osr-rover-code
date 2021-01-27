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
    roboclaw = Roboclaw("/dev/serial0", 115200)
    try:
        assert roboclaw.Open() == 1
    except AssertionError as e:
        raise e("Could not open comport /dev/serial0, make sure it has the correct permissions and is available")

    print(roboclaw.ReadVersion(address))
    print(roboclaw.ReadEncM1(address))
