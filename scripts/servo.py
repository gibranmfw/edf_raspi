#! usr/bin/env python
import rospy
import time
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode

class ModeHandler:
	def __init__(self):
		pass

	def change_mode(self, mode):
		rospy.wait_for_service("/mavros/set_mode")
		cm = rospy.ServiceProxy("mavros/set_mode", SetMode)
		resp1 = cm(custom_mode=mode)
		return "True" in str(resp1)

class ServoHandler(object):
	def __init__(self, channel):
		self.channel = channel
		self.exec_time = 1
		self.max_power = 2000
		self.min_power = 1100
		self.mode_handler = ModeHandler()

	def set_max_power(self, power):
		self.max_power = power

	def set_min_power(self, power):
		self.min_power = power

	def move_custom(self, power):
		success = self.mode_handler.change_mode("manual")
		if success:
			self.__move_servo(power)
	
	def move_max(self):
		success = self.mode_handler.change_mode("manual")
		if success:
			self.__move_servo(self.max_power)

	def move_min(self):
		success = self.mode_handler.change_mode("manual")
		if success:
			self.__move_servo(self.min_power)

	def __move_servo(self, pwm):
		pub = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size = 10)
		r = rospy.Rate(10)
		msg = OverrideRCIn()
		start = time.time()
		flag = True
		msg.channels[self.channel] = pwm
		while not rospy.is_shutdown() and flag:
			sample_time = time.time()
			if((sample_time - start) > self.exec_time):
				flag = False
				rospy.loginfo(msg)
				pub.publish(msg)
				r.sleep()

class ParachuteHandler(ServoHandler):
	def __init__(self, channel):
		super(ParachuteHandler, self).__init__(channel)

class MotorHandler(ServoHandler):
	def __init__(self, channel):
		super(MotorHandler, self).__init__(channel)
		self.set_max_power(2000)
		self.set_min_power(700)

	def move_custom(self, power):
		super(MotorHandler, self).move_custom(power)
		self.mode_handler.change("rtl")

	def move_min(self):
		super(MotorHandler, self).move_min()
		self.mode_handler.change_mode("rtl")

	def move_max(self):
		super(MotorHandler, self).move_max()
		self.mode_handler.change_mode("rtl")

