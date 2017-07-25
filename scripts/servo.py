import rospy
import time
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode

class Servo:
	def __init__(self, channel):
		self.channel = channel
		self.exec_time = 1
		self.max_power = 2000
		self.min_power = 1100
	
	def move_max(self):
		rospy.wait_for_service("/mavros/set_mode")
		change_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)
		resp1 = change_mode(custom_mode="manual")
		if "True" in str(resp1):
			try:
				self.move_servo(self.max_power)
			except rospy.ROSInterruptionException, e:
				print(e)

	def move_min(self):
		rospy.wait_for_service("/mavros/set_mode")
		change_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)
		resp1 = change_mode(custom_mode="manual")
		if "True" in str(resp1):
			try:
				self.move_servo(self.min_power)
			except rospy.ROSInterruptionException, e:
				print(e)

	def move_servo(self, pwm):
		pub = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size = 10)
		rospy.init_node("custmom_talker", anonymous=True)
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
