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
	def __init__(self, channel1, channel2 = -1):
		self.channel1 = channel1
		self.channel2 = channel2
		self.exec_time = 1
		self.max_power = 2000
		self.min_power = 800
		self.mode_handler = ModeHandler()

	def set_max_power(self, power):
		self.max_power = power

	def set_min_power(self, power):
		self.min_power = power

	def move_custom(self, power1 = 65535, power2 = 65535):
		success = self.mode_handler.change_mode("manual")
		if success:
			self.__move_servo(pwm1=power1, pwm2=power2)
	
	def move1_max(self):
		success = self.mode_handler.change_mode("manual")
		if success:
			self.__move_servo(pwm1=self.max_power)

	def move2_max(self):
		success = self.homde_handler.change_mode("manual")
		if(success):
			self.__move_servo(pwm2=self.max_power)

	def move1_min(self):
		success = self.mode_handler.change_mode("manual")
		if success:
			self.__move_servo(pwm1=self.min_power)
	
	def move2_min(self):
		success = self.mode_handler.change_mode("manual")
		if success:
			self.__move_servo(pwm2=self.min_power)	

	def __move_servo(self, pwm1 = 65535, pwm2 = 65535):
		pub = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size = 10)
		r = rospy.Rate(10)
		msg = OverrideRCIn()
		start = time.time()
		flag = True
		msg.channels[self.channel1] = pwm1
		if(channel2 != -1):
			msg.channels[self.channel2] = pwm2
		#rospy.loginfo(msg)
		#pub.publish(msg)
		while not rospy.is_shutdown() and flag:
			sample_time = time.time()
			if((sample_time - start) > self.exec_time):
				flag = False
				rospy.loginfo(msg)
				pub.publish(msg)
				r.sleep()

