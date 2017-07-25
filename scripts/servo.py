import rospy
import time
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode

class Servo:
	def __init__(channel):
		self.channel = channel
		exec_time = 1

	def move_para(pwm):
		pub = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size = 10)
		rospy.init_node("custmom_talker", anonymous=True)
		r = rospy.Rate(10)
		msg = OverrideRCIn()
		start = time.time()
		flag = True
		msg.channels[self.channels] = pwm
		while not rospy.is_shutdown() and flag:
			sample_time = time.time()
			if((sample_time - start) > exec_time):
				flag = False
				rospy.loginfo(msg)
				pub.publish(msg)
				r.sleep()


