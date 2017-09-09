"""
This file contains basic class to control
the communication using xbee
"""

#! /usr/bin/python

import serial
from time import sleep

class CommunicationHandler:
	"""
    This class handles communication using serial
    """
	def __init__(self):
		"""
        __init__ method of CommunicationHandler class,
        it open a serial port in /dev/ttyUSB0

        :return: None
        """
		self.ser = serial.Serial("/dev/ttyUSB0", 57600, timeout=1)

	def read(self):
		"""
        Read 1 line from serial

        :return: None
        """
		return self.ser.readline()

	def write(self, message):
		"""
        Write 1 line to serial

        :message: String, data to write
        :return: None
        """
		self.ser.writelines(message)
	
	def close(self):
		"""
        close the serial port

        :return: None
        """
		self.ser.close()

