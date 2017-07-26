#! /usr/bin/env python

import rospy
from servo import ParachuteHandler
from servo import MotorHandler

class SafetyHandler:
	def __init__(self):
		self.emergency = False
		self.para = ParachuteHandler()
		self.esc = MotorHandler()
		self.safe_range = 200 #200 meter
		self.motoroff_range = 175

	def on(self):
		self.emergency = True

	def off(self):
		self.emergency = False

	def check_emergency(self):
		if(self.emergency):
			self.para.move_max()
			self.esc.move_min()

	def check_range(self, range):
		if(range >= self.motoroff_range):
			self.esc.move_min()

		if(range >= self.safe_range):
			self.esc.move_min()
			self.para.move_max()
