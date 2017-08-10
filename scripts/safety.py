#! /usr/bin/env python

import rospy
from servo import ParachuteHandler
from servo import MotorHandler

class SafetyHandler:
	def __init__(self):
		self.para = ParachuteHandler(4)
		self.esc = MotorHandler(2)
		self.safe_range = 200 #200 meter
		self.motoroff_range = 175

	def emergency(self):
		self.para.move_max()
		self.esc.move_min()

	def check_range(self, range):
		if(range >= self.motoroff_range):
			self.esc.move_min()

		if(range >= self.safe_range):
			self.esc.move_min()
			self.para.move_max()
