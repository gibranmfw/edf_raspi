"""
This file contains rocket safety features
"""

#! /usr/bin/env python

import rospy
from servo import ServoHandler

class SafetyHandler:
	"""
    This class contains functions to ensure
    the safety of the rocket
    """

	def __init__(self, sh):
		"""
        __init__ method of SafetyHandler class

		:sh: ActuatorHandler, to turn off motor and open the parachute
        :return: None
        """
		self.act = sh
		self.safe_range = 200 #200 meter
		self.motoroff_range = 175
		self.para_flag = False

	def toggle_para(self):
		"""
        toggle parachute

        :return: None
        """
		if(self.para_flag):
			self.act.move2_max()
			self.para_flag = False
		else:
			self.act.move2_min()
			self.para_flag = True

	def emergency(self):
		"""
        toggle parachute and turn off motor

        :return: None
        """
		if(self.para_flag):
			self.act.move_custom(power1 = 2000, power2 = 800)
			self.para_flag = False
		else:
			self.act.move_custom(power1 = 800, power2 = 800)
			self.para_flag = True

	def check_range(self, range):
		"""
        check current distance, 
        if it exceed 200 m turn on the safety features
	
		:range: current distance
        :return: None
        """
		print(range)
		if(range >= self.motoroff_range):
			self.act.move1_min()

		if(range >= self.safe_range):
			self.emergency()
