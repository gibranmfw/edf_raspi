"""
This file contains definition of
ros node to control the actuator
"""

#! /usr/bin/env python
import rospy
import message_filters
from utils.communication import CommunicationHandler as ch
from utils.safety import SafetyHandler
from utils.servo import ServoHandler
from utils.data import GPSHandler
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from std_msgs.msg import String

class ActuatorHandler:
	"""
    This class handles all actuator system of the rocket
    """
	def __init__(self):
		"""
        __init__ method of ActuatorHandler class
        used output channel 6 as parachute, and output channel 7 as motor

        :return: None
        """
		self.pub = rospy.Publisher("/aurora/senddata", String, queue_size=50)
		self.esc = ServoHandler(5, 6)
		self.sh = SafetyHandler(self.esc)
		self.comm = ch()
		self.fetch_flag = True
		self.toggle = True
		self.slat = 0
		self.slong = 0
		self.check_home = False
		self.motor_flag = True
	
	def set_alt(self, alt):
		"""
        set current altitude

		:alt: current altitude
        :return: None
        """
		self.curr_alt = alt

	def toggle_flag(self):
		"""
        set fetch_flag to True

        :return: None
        """
		self.fetch_flag = True

	def fetch_data(self, gh):
		"""
        fetch gps data from APM, it also set
        current coordinate as home position

		:gh: GPSHandler
        :return: None
        """
		if(self.fetch_flag):
			self.slat, self.slong = gh.get_latlong()
			self.fetch_flag = False
		else:
			data = gh.get_data_w_dis(self.slat, self.slong)
			temp = data.split(" ")
			self.curr_dis = float(temp[2])
			if(self.check_home):
				self.sh.check_range(self.curr_dis)
				self.sh.check_range(self.curr_alt)
			data = "{} {}".format(data, self.curr_alt)
			print(data)
			self.pub.publish(data)
			rospy.loginfo(data)

	def toggle_motor(self):
		"""
        toggle motor on and off

        :return: None
        """
		if(self.motor_flag):
			self.esc.move2_max()
			self.motor_flag = False
		else:
			self.esc.move2_min()
			self.motor_flag = True

	def __callback(self, data):
		"""
        callback function, it determines how the program should do
        based on the subscriber

		:data: ros Subscriber object, it vary based on the Subscriber
        :return: None
        """
		if(type(data) == Float64):
			self.set_alt(data.data)
		elif(type(data) == String):
			if(data.data == "1\n"):
				print("parachute on")
				self.sh.emergency()
			elif(data.data == "2\n"):
				print("toggle motor")
				self.toggle_motor()
			elif(data.data == "3\n"):
				print("fetching home position")
				self.toggle_flag()
				self.check_home = True
		else:
			gh = GPSHandler(data)
			self.fetch_data(gh)

	def start(self):
		"""
        start the actuator node program, this method never stop until
        the user terminate the program

        :return: None
        """
		rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.__callback)
		rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self.__callback)
		rospy.Subscriber("/aurora/gcsdata", String, self.__callback)
		rospy.spin()

def start():
	"""
	init ros node, and start the program

    :return: None
    """
	rospy.init_node('actuator_node', anonymous=True)
	ah = ActuatorHandler()
	ah.start()

if __name__ == '__main__':
	start()
