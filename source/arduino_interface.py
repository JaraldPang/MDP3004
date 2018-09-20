import os
import serial
#import time


class ArduinoComms():
	def __init__(self):
		if os.path.exists('/dev/ttyACM0') == True:
			ser = serial.Serial('/dev/ttyACM0', 115200)
		else
			ser = serial.Serial('/dev/ttyACM1', 115200)
	
	def writeToArduino(self, msg):
		ser.write((str(msg).encode('UTF-8'))) #serial comms need to encode then can send
		
	def readFromArduino(self):
		read_serial=ser.readline().decode('UTF-8').rstrip('\r\n') #aruino using println to send so need remove \r\n
		return read_serial
