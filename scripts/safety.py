#! /usr/bin/env python

import rospy
from servo import ParachuteHandler
from servo import MotorHandler

class SafetyHandler:
	def __init__(self):
		self.emergency = False
		self.para = ParachuteHandler()
		self.esc = MotorHandler()

	def on(self):
		self.emergency = True

	def off(self):
		self.emergency = False

	def check_emergency(self):
		if(self.emergency):
			self.para.move_max()
			self.esc.move_min()
