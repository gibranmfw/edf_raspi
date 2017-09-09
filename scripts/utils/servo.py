"""
This file contains how raspy control
the servos manually
"""

#! usr/bin/env python
import rospy
import time
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode

class ModeHandler:
	"""
    This class handle how to change fly mode
    of APM
    """
	def __init__(self):
		"""
        __init__ method of ModeHandler class

        :return: None
        """
		pass

	def change_mode(self, mode):
		"""
        change current fly mode

		:mode: String, fly mode that want to set
        :return: boolean, true if success, false if failed
        """
		rospy.wait_for_service("/mavros/set_mode")
		cm = rospy.ServiceProxy("mavros/set_mode", SetMode)
		resp1 = cm(custom_mode=mode)
		return "True" in str(resp1)

class ServoHandler(object):
	"""
    This class handle how to control servos
    """
	def __init__(self, channel1, channel2 = -1):
		"""
        __init__ method of ServoHandler class
        currently it only handle 2 channel of servo

		:channel1: int, output channel of apm - 1
		:channel2: int, output channel of apm - 1
        :return: None
        """
		self.channel1 = channel1
		self.channel2 = channel2
		self.exec_time = 1
		self.max_power = 2000
		self.min_power = 800
		self.same = 65535
		self.mode_handler = ModeHandler()

	def set_max_power(self, power):
		"""
        set max pwm

		:power: int, pwm value
        :return: None
        """
		self.max_power = power

	def set_min_power(self, power):
		"""
        set min pwm

		:power: int, pwm value
        :return: None
        """
		self.min_power = power

	def move_custom(self, power1 = self.same, power2 = self.same):
		"""
        move servo

		:power1: int, pwm value, default 65535
		:power2: int, pwm value, default 65535
        :return: None
        """
		success = self.mode_handler.change_mode("manual")
		if success:
			self.__move_servo(pwm1=power1, pwm2=power2)
	
	def move1_max(self):
		"""
        set pwm servo in channel1 to max

        :return: None
        """
		success = self.mode_handler.change_mode("manual")
		if success:
			self.__move_servo(pwm1=self.max_power)

	def move2_max(self):
		"""
        set pwm servo in channel2 to max

        :return: None
        """
		success = self.mode_handler.change_mode("manual")
		if(success):
			self.__move_servo(pwm2=self.max_power)

	def move1_min(self):
		"""
        set pwm servo in channel1 to min

        :return: None
        """
		success = self.mode_handler.change_mode("manual")
		if success:
			self.__move_servo(pwm1=self.min_power)
	
	def move2_min(self):
		"""
        set pwm servo in channel2 to min

        :return: None
        """
		success = self.mode_handler.change_mode("manual")
		if success:
			self.__move_servo(pwm2=self.min_power)	

	def __move_servo(self, pwm1 = self.same, pwm2 = self.same):
		"""
        move servo using rcoverride

		:pwm1: int, pwm value for channel1, default 65535
		:pwm2: int, pwm value for channel2, default 65535
        :return: None
        """
		pub = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size = 10)
		r = rospy.Rate(10)
		msg = OverrideRCIn()
		start = time.time()
		flag = True
		msg.channels[self.channel1] = pwm1
		if(self.channel2 != -1):
			msg.channels[self.channel2] = pwm2
		while not rospy.is_shutdown() and flag:
			sample_time = time.time()
			if((sample_time - start) > self.exec_time):
				flag = False
				rospy.loginfo(msg)
				pub.publish(msg)
				r.sleep()
		self.mode_handler.change_mode("auto")

