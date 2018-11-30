#!/usr/bin/env python
import time
from osr import Rover
from arguments import Arguments
import json

'''
This code runs the JPL Open Source Rover. It accepts a few command line arguments for different functionality
   -s : Attempts to connect to a Unix socket for controlling the LED screen. The screen.py script must be running
   			previous to this in order to work. It lives at ../led/screen.py
   -x : Attemps to connect to a remote Xbox Controller to recieve drive commands
   -b : Attmpts to connect to a remote Bluetooth device to recieve drive commands

An example line running this script to run the LED screen and with an Xbox controller
	sudo python main.py -s -x
'''


def main():
	args = Arguments()
	with open('/home/pi/osr/rover/config.json') as f:
		config = json.load(f)

	rover = Rover(config,args.bt_flag,args.xbox_flag,args.unix_flag)
	
	while True:
		try:
			rover.drive()
			time.sleep(0.1)

		except Exception as e:
			print e
			rover.cleanup()
			time.sleep(0.5)
			rover.connectController()

if __name__ == '__main__':
	main()
