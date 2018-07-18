#!/usr/bin/env python
import time
import math
from motor_controller import Motor

class Robot(Motor):
	'''
	Robot class contains all the math and motor control algorithms to move the rover

	In order to call command the robot the only method necessary is the sendCommands() method
	
	with drive velocity and turning amount
	
	:param class object Motor : Motor class for communication to the motor controllers

	'''
	def __init__(self,config):
		super(Robot,self).__init__(config)
		self.d1 = config['MECHANICAL_CONFIG']['d1']
		self.d2 = config['MECHANICAL_CONFIG']['d2']
		self.d3 = config['MECHANICAL_CONFIG']['d3']
		self.d4 = config['MECHANICAL_CONFIG']['d4']
		self.killMotors()

	@staticmethod	
	def tick2deg(tick,e_min,e_max):
		'''
		Converts a tick to physical degrees
		
		:param int tick : Current encoder tick
		:param int e_min: The minimum encoder value based on physical stop
		:param int e_max: The maximum encoder value based on physical stop
		
		'''
		return (tick - (e_max + e_min)/2.0) * (90.0/(e_max - e_min))
		
	@staticmethod
	def deg2tick(deg,e_min,e_max):
		'''
		Converts a degrees to tick value
		
		:param int deg  : Degrees value desired
		:param int e_min: The minimum encoder value based on physical stop
		:param int e_max: The maximum encoder value based on physical stop
		
		'''
		return (e_max + e_min)/2 + ((e_max - e_min)/90)*deg	
	
	def calculateVelocity(self,v,r):
		'''
		Returns a list of speeds for each individual drive motor based on current turning radius

		:param int v: Drive speed command range from -100 to 100
		:param int r: Turning radius command range from -100 to 100

		'''
		v *= 0.01
		if (v == 0):
			return [v]*6

		if (r == 0 or r >= 250 or r <= -250):
				return [v] * 6                        # No turning radius, all wheels same speed
		else:
			x = v/((abs(r) + self.d4))                   # Wheels can't move faster than max (127)
			a = math.pow(self.d2,2)
			b = math.pow(self.d3,2)
			c = math.pow(abs(r) + self.d1,2)
			d = math.pow(abs(r) - self.d1,2)
			e = abs(r) - self.d4

			v1 = x*math.sqrt(b + d)
			v2 = x*e                            # Slowest wheel
			v3 = x*math.sqrt(a + d)
			v4 = x*math.sqrt(a + c)
			v5 = v                           # Fastest wheel
			v6 = x*math.sqrt(b + c)

			v1 = int(v1*100)/100.0
			v2 = int(v2*100)/100.0
			v3 = int(v3*100)/100.0
			v4 = int(v4*100)/100.0
			v5 = int(v5*100)/100.0
			v6 = int(v6*100)/100.0

			if (r < 0):
				velocity = [v1,v2,v3,v4,v5,v6]
			elif (r > 0):
				velocity = [v6,v5,v4,v3,v2,v1]
			return velocity

	def calculateTargetDeg(self,radius):
		'''
		Takes a turning radius and calculates what angle [degrees] each corner should be at

		:param int radius: Radius drive command, ranges from -100 (turning left) to 100 (turning right)

		'''
		#Scaled from 250 to 20 inches. For more information on these numbers look at the Software Controls.pdf
		if radius == 0: r = 250
		elif -100 <= radius <= 100: r = 220 - abs(radius)*(250/100)
		else: r = 250
		
		if r == 250: return [0]*4

		ang1 = int(math.degrees(math.atan(self.d1/(abs(r)+self.d3))))
		ang2 = int(math.degrees(math.atan(self.d2/(abs(r)+self.d3))))
		ang3 = int(math.degrees(math.atan(self.d2/(abs(r)-self.d3))))
		ang4 = int(math.degrees(math.atan(self.d1/(abs(r)-self.d3))))

		if radius > 0: return [ang2,-ang1,-ang4,ang3]
		else: return [-ang4,ang3,ang2,-ang1]	

	def getCornerEnc(self):
		'''
		Returns a list of the tick value of each corner encoder
		
		'''
		enc = []
		for i in range(4):
			index = int(math.ceil((i+1)/2.0)+2)
			if not (i % 2):
				enc.append(self.rc.ReadEncM1(self.address[index])[1])
			else:
				enc.append(self.rc.ReadEncM2(self.address[index])[1])
		return enc
	
	def getCornerDeg(self):
		'''
		Returns a list of what degrees each corner currently is at

		'''
		enc = self.getCornerEnc()
		deg = [None] *4
		
		for i in range(4):
			deg[i] = int(self.tick2deg(enc[i],self.enc_min[i],self.enc_max[i]))
		return deg

	def approxTurningRadius(self,enc):
		'''
		Takes the list of current corner angles and approximates the current turning radius [inches]

		:param list [int] enc: List of encoder ticks for each corner motor

		'''
		if enc[0] == None:
			return 250
		try:
			if enc[0] > 0:
				r1 = (self.d1/math.tan(math.radians(abs(enc[0])))) + self.d3
				r2 = (self.d2/math.tan(math.radians(abs(enc[1])))) + self.d3
				r3 = (self.d2/math.tan(math.radians(abs(enc[2])))) - self.d3
				r4 = (self.d1/math.tan(math.radians(abs(enc[3])))) - self.d3
			else:
				r1 = -(self.d1/math.tan(math.radians(abs(enc[0])))) - self.d3
				r2 = -(self.d2/math.tan(math.radians(abs(enc[1])))) - self.d3
				r3 = -(self.d2/math.tan(math.radians(abs(enc[2])))) + self.d3
				r4 = -(self.d1/math.tan(math.radians(abs(enc[3])))) + self.d3
			radius = (r1 + r2 + r3 + r4)/4
			return radius
		except:
			return 250

	def cornerPosControl(self, tar_enc):
		'''
		Takes the target angle and gets what encoder tick that value is for position control

		:param list [int] tar_enc: List of target angles in degrees for each corner
		'''
		tick = []
		for i in range(4):
			tick.append(self.deg2tick(tar_enc[i],self.enc_min[i],self.enc_max[i]))
		
		enc = self.getCornerEnc()
		for i in range(4):
			if abs(tick[i] - enc[i]) < 30: tick[i] = 0          # stopping the motor when it is close to target reduced motor jitter
		
		self.cornerToPosition(tick)
		
	def sendCommands(self,v,r):
		'''
		Driving method for the Rover, rover will not do any commands if any motor controller
		throws an error

		:param int v: driving velocity command, % based from -100 (backward) to 100 (forward)
		:param int r: driving turning radius command, % based from -100 (left) to 100 (right)

		'''
		current_radius = self.approxTurningRadius(self.getCornerDeg())
		velocity = self.calculateVelocity(v,current_radius)
		self.cornerPosControl(self.calculateTargetDeg(r))
		for i in range(6):
			self.sendMotorDuty(i,velocity[i])


