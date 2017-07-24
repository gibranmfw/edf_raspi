#! /usr/bin/python

import serial

class Xbee:
	def __init__():
		this.ser = serial.Serial("/dev/ttyAMA0", baudrate=56700)
	
	def write(data):
		ser.write(str(data))

	def read():
		return ser.read()
