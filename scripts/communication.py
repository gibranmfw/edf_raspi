#! /usr/bin/python

import serial
from time import sleep

class CommunicationHandler:
	def __init__(self):
		self.ser = serial.Serial("/dev/ttyUSB0", 57600, timeout=1)

	def read(self):
		return self.ser.readline()

	def write(self, message):
		ser.writeline(message)
		sleep(0.05)
	
	def close(self):
		self.ser.close()

