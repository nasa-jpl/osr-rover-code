#!/usr/bin/env python
import serial
import time

PREAMBLE_LOW = 0xCD
PREAMBLE_HIGH = 0xAB
CONNECTED = 0x01
DISCONNECTED = 0x02

LOW_MASK = 0xFF00
HIGH_MASK = 0x00FF

CONN      = 2
BATT      = 3
STATUS    = 4
TEMP      = 5
DRIVE_CUR = 8
STEER_CUR = 11
FACE      = 13
CHKSUM    = 14

class LedScreen():
	def __init__(self):
		dev = "/dev/ttyUSB0"
		baud = 9600
		self.ser = serial.Serial(dev,baud,timeout=1.0)
		self.ser.flushInput()

		self.out_msg = [-10] * 16
		self.in_msg  = [-10] * 16


		self.b_low = 12.7
		self.b_high = 16.7
		self.b_range = self.b_high - self.b_low
		
		self.t_low   = 28
		self.t_high  = 40
		self.t_range = self.t_high - self.t_low
		
		
		self.c_low  = 0
		self.c_high = 1
		self.c_range = self.c_high - self.c_low
		
		self.out_msg[0] = PREAMBLE_HIGH
		self.out_msg[1] = PREAMBLE_LOW
		
		self.send_init()

	def build_msg(self,connected,battery,error,temp,currents):
		self.connected_check(1)
		self.battery_check(battery)
		self.error_check(error)
		self.temp_check(temp)
		self.drive_current_check(currents)
		self.steering_current_check(currents)
		self.out_msg[FACE] = 0x01
		self.build_chksum()
		print self.out_msg
		self.send_msg()
		
	def connected_check(self,connected):
		if (connected):
			self.out_msg[CONN] = 0x01
		else:
			self.out_msg[CONN] = 0x00
		
	def battery_check(self,battery):
		battery *= 0.1 
		if   (self.b_high - (self.b_range * 1/5) <= battery):
			self.out_msg[BATT] = 0x1F
		elif (self.b_high - (self.b_range * 2/5) <= battery):
			self.out_msg[BATT] = 0x0F
		elif (self.b_high - (self.b_range * 3/5) <= battery):
			self.out_msg[BATT] = 0x07
		elif (self.b_high - (self.b_range * 4/5) <= battery):
			self.out_msg[BATT] = 0x03
		elif (self.b_high - (self.b_range * 5/5) <= battery):
			self.out_msg[BATT] = 0x01
		else:
			self.out_msg[BATT] = 0x00

	def error_check(self,error):
		result = 0x00
		for i in range(5):
			if error[i]:
				result = result | (0x01 << (5-i))
		self.out_msg[STATUS] = result

	def temp_check(self,temp):
		temperature = [[0,temp[0]],temp[1:3],temp[3:]]
		for i in range(3):
			result = 0
			for j in range(2):
				val = temperature[i][j] * 0.1
				if val == 0:
					tmp = 0x00
				else:
					if   (self.t_high - (self.t_range * 1/5) <= val):
						tmp = 0x4
					elif (self.t_high - (self.t_range * 2/5) <= val):
						tmp = 0x3
					elif (self.t_high - (self.t_range * 3/5) <= val):
						tmp = 0x2
					elif (self.t_high - (self.t_range * 4/5) <= val):
						tmp = 0x1
					else:
						tmp = 0x0
				tmp = tmp << (4 - (4*j))
				result += tmp
			self.out_msg[TEMP + i] = result

	def drive_current_check(self,cur):
		currents = [cur[0:2],cur[2:4],cur[4:6]]
		for i in range(3):
			result = 0
			for j in range(2):
				val = currents[i][j] * 0.1
				if   (self.c_high - (self.c_range * 1/5) <= val):
					tmp = 0x4
				elif (self.c_high - (self.c_range * 2/5) <= val):
					tmp = 0x3
				elif (self.c_high - (self.c_range * 3/5) <= val):
					tmp = 0x2
				elif (self.c_high - (self.c_range * 4/5) <= val):
					tmp = 0x1
				else:
					tmp = 0x0
				
				tmp = tmp << (4 - (4*j))
				result += tmp
			self.out_msg[DRIVE_CUR + i] = result
	
	def steering_current_check(self,cur):
		currents = [cur[6:8],cur[8:]]
		for i in range(2):
			result = 0
			for j in range(2):
				val = currents[i][j] * 0.1
				if   (self.c_high - (self.c_range * 1/5) <= val):
					tmp = 0x4
				elif (self.c_high - (self.c_range * 2/5) <= val):
					tmp = 0x3
				elif (self.c_high - (self.c_range * 3/5) <= val):
					tmp = 0x2
				elif (self.c_high - (self.c_range * 4/5) <= val):
					tmp = 0x1
				else:
					tmp = 0x0
				tmp = tmp << (4 - (4*j))
				result += tmp
			self.out_msg[STEER_CUR + i] = result		
	
	def transistion_to_idle(self):
		self.out_msg[CONN] = 0xFF
		self.build_chksum()
		self.send_msg()
		
	def send_msg(self):
		message = bytearray()
		for val in self.out_msg:
			message.append(val)
		self.ser.write(message)

	def populate_test_vals(self):
		self.out_msg[0] = PREAMBLE_HIGH
		self.out_msg[1] = PREAMBLE_LOW
		self.out_msg[2] = CONNECTED
		self.out_msg[3] = 0x1F
		self.out_msg[4] = 0x00
		self.out_msg[5] = 0x01
		self.out_msg[6] = 0x23
		self.out_msg[7] = 0x44
		self.out_msg[8] = 0x12
		self.out_msg[9] = 0x34
		self.out_msg[10] = 0x12
		self.out_msg[11] = 0x34
		self.out_msg[12] = 0x12
		self.out_msg[13] = 0x34

	def build_chksum(self):
		chk = 0
		for i in range(CONN,CHKSUM):
			chk += self.out_msg[i]

		#print chk
		chk_low  =  chk & HIGH_MASK
		chk_high = (chk & LOW_MASK ) >> 8
		#print chk_high,chk_low
		self.out_msg[CHKSUM] = chk_high
		self.out_msg[CHKSUM + 1] = chk_low

	def check_for_afffirm(self):

		affirm = self.ser.read(2)
		print affirm

	def send_init(self):
		while True:
			init_packet = bytearray()
			init_packet.append(0xA)
			#print "init_packet",ord(init_packet)
			self.ser.write(init_packet)
			response = self.ser.read(2)
			if (ord(response[0]) == 67 and ord(response[1]) == 68):
				print "correct response received!"
				break
			time.sleep(0.5)

def main():
	led = LedScreen()
	
	while True:
		#led.read_serial()
		led.send_msg()
		time.sleep(1)



if __name__ == '__main__':
	main()




