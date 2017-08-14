#! /usr/bin/env python

import rospy
from servo import ServoHandler

class SafetyHandler:
	def __init__(self, sh):
		self.act = sh
		self.safe_range = 200 #200 meter
		self.motoroff_range = 175
		self.para_flag = False

	def toggle_para():
		if(self.para_flag):
			self.act.move2_max()
			self.para_flag = False
		else:
			self.act.move2_min()
			self.para_flag = True

	def emergency(self):
		if(self.para_flag):
			self.act.move_custom(power1=800, power2=2000)
			self.para_flag = False
		else:
			self.act.move_custom(power1=800, power2=800)
			self.para_flag  = True

	def check_range(self, range):
		print(range)
		if(range >= self.motoroff_range):
			self.act.move1_min()

		if(range >= self.safe_range):
			self.emergency()
