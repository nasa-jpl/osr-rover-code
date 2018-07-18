import time
import serial
import threading

class Threads():
	def __init__(self):
		self.kill = 1
		ser = serial.Serial(
			port='/dev/ttyS0',
			baudrate=9600,
			parity=serial.PARITY_NONE,
			stopbits=serial.STOPBITS_ONE,
			bytesize=serial.EIGHTBITS,
			timeout=1
			)

def SendData(self):
	counter=0
	while self.kill:
		socket.write(str(counter))
		time.sleep(1)
		counter +=1

def ReadData(self):
	counter=0
	while 1:
		try:
			x=self.ser.readline()
			print 'Sucessful data recieved over serial!'
			counter+=1
		except:
			print 'No serial data read, check wiring and software steps'
			self.kill=0
			break
		if counter >= 3:
			print "Serial communication verified! \nExiting"
			self.kill=0
			break

def main():
	mythread = Threads()

	clientThread = threading.Thread(target=mythread.SendData, args=())
	clientThread.start()

	mythread.ReadData()

if __name__ == '__main__':
	main()