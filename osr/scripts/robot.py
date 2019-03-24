
#!/usr/bin/env python
import rospy
import time
import math

class Robot():
	'''
	Robot class contains all the math and motor control algorithms to move the rover

	In order to call command the robot the only method necessary is the sendCommands() method

	with drive velocity and turning amount


	'''
	def __init__(self):
		distances = rospy.get_param('mech_dist','7.254,10.5,10.5,10.073').split(",")
		self.d1 = float(distances[0])
		self.d2 = float(distances[1])
		self.d3 = float(distances[2])
		self.d4 = float(distances[3])

		#to change when getting Mc thread lock to work
		enc_min_raw = rospy.get_param('enc_min').split(',')
		enc_max_raw = rospy.get_param('enc_max').split(',')
		enc_min_int = [None]*4
		enc_max_int = [None]*4
		for i in range(4):
			enc_min_int[i] = int(enc_min_raw[i])
			enc_max_int[i] = int(enc_max_raw[i])

		self.enc_min = enc_min_int
		self.enc_max = enc_max_int
		self.mids = [None]*4
		for i in range(4):
			self.mids[i] = (self.enc_max[i] + self.enc_min[i])/2

	@staticmethod
	def tick2deg(tick,e_min,e_max):
		'''
		Converts a tick to physical degrees

		:param int tick : Current encoder tick
		:param int e_min: The minimum encoder value based on physical stop
		:param int e_max: The maximum encoder value based on physical stop

		'''
		try:
			temp =(tick - (e_max + e_min)/2.0) * (90.0/(e_max - e_min))
			return temp
		except:
			return 0

	@staticmethod
	def deg2tick(deg,e_min,e_max):
		'''
		Converts a degrees to tick value

		:param int deg  : Degrees value desired
		:param int e_min: The minimum encoder value based on physical stop
		:param int e_max: The maximum encoder value based on physical stop

		'''
		temp = (e_max + e_min)/2 + ((e_max - e_min)/90)*deg
		if temp < e_min: temp = e_min
		elif temp > e_max: temp = e_max
		print temp,e_max,e_min,deg
		return temp

	def calculateVelocity(self,v,r):
		'''
		Returns a list of speeds for each individual drive motor based on current turning radius

		:param int v: Drive speed command range from -100 to 100
		:param int r: Current turning radius range from -250 to 250

		'''

		if (v == 0):
			return [0]*6

		if (abs(r) <= 5):
			return [v] * 6                        # No turning radius, all wheels same speed
		else:

			radius = 250 - (230 * abs(r))/100.0
			if radius < 0: rmax *= -1
			rmax = radius + self.d4

			a = math.pow(self.d2,2)
			b = math.pow(self.d3,2)
			c = math.pow(abs(radius) + self.d1,2)
			d = math.pow(abs(radius) - self.d1,2)
			e = abs(radius) - self.d4
			rmax_float = float(rmax)

			v1 = int(v*(math.sqrt(b + d))/rmax_float)
			v2 = int((v*e/rmax_float))                        # Slowest wheel
			v3 = int((v*math.sqrt(a + d))/rmax_float)
			v4 = int((v*math.sqrt(a + c))/rmax_float)
			v5 = int(v)                            # Fastest wheel
			v6 = int((v*math.sqrt(b + c))/rmax_float)

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

		angles = [ang1,ang2,ang3,ang4]
		#rospy.loginfo(angles)
		for i in range(4):
			if   angles[i] < -45: angles[i] = -43
			elif angles[i] >  45: angles[i] =  43

		if radius > 0:

			return [ang2,-ang1,-ang4,ang3]
		else:

			return [-ang4,ang3,ang2,-ang1]

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

	def getCornerDeg(self,enc):
		'''
		Returns a list of what degrees each corner currently is at

		'''
		deg = [None] *4

		for i in range(4):
			deg[i] = int(self.tick2deg(enc[i],self.enc_min[i],self.enc_max[i]))
			#rospy.loginfo(deg)
		return deg

	def approxTurningRadius(self,deg):
		'''
		Takes the list of current corner angles and approximates the current turning radius [inches]

		:param list [int] enc: List of encoder ticks for each corner motor

		'''
		if deg[0] == None:
			return 250

		## Need to fix the try except clause for when all the deg are exactly 0
		if deg == [0,0,0,0]:
			return 250
		try:
			if deg[0] >= 0:
				r1 = (self.d1/math.tan(math.radians(abs(deg[0])))) + self.d3
				r2 = (self.d2/math.tan(math.radians(abs(deg[1])))) + self.d3
				r3 = (self.d2/math.tan(math.radians(abs(deg[2])))) - self.d3
				r4 = (self.d1/math.tan(math.radians(abs(deg[3])))) - self.d3
			else:
				r1 = -(self.d1/math.tan(math.radians(abs(deg[0])))) - self.d3
				r2 = -(self.d2/math.tan(math.radians(abs(deg[1])))) - self.d3
				r3 = -(self.d2/math.tan(math.radians(abs(deg[2])))) + self.d3
				r4 = -(self.d1/math.tan(math.radians(abs(deg[3])))) + self.d3
			radius = (r1 + r2 + r3 + r4)/4

			return radius
		except:
			return 250

	def calculateTargetTick(self, tar_enc, cur_enc):
		'''
		Takes the target angle and gets what encoder tick that value is for position control

		:param list [int] tar_enc: List of target angles in degrees for each corner
		'''
		tick = []
		for i in range(4):
			tick.append(self.deg2tick(tar_enc[i],self.enc_min[i],self.enc_max[i]))

		for i in range(4):
			if abs(tick[i] - cur_enc[i]) < 25:
				#rospy.loginfo(str(i) + ", " + str(tick[i]) + ", " + str(cur_enc[i]) + ", " + str(self.mids[i]))		 
				tick[i] = -1          # stopping the motor when it is close to target reduced motor jitter
		print tick
		#rospy.loginfo(tick)
		return tick

	def generateCommands(self,v,r,encs):
		'''
		Driving method for the Rover, rover will not do any commands if any motor controller
		throws an error

		:param int v: driving velocity command, % based from -100 (backward) to 100 (forward)
		:param int r: driving turning radius command, % based from -100 (left) to 100 (right)

		'''
		#fix params list above ^^
		#cur_radius = self.approxTurningRadius(self.getCornerDeg(encs))
		velocity   = self.calculateVelocity(v,r)
		ticks      = self.calculateTargetTick(self.calculateTargetDeg(r),encs)

		return (velocity,ticks)


