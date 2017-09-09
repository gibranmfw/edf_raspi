"""
This file contains definition of
ros node to control IMU data
"""

#! /usr/bin/env python
import rospy 
import message_filters 
from utils.communication import CommunicationHandler as ch 
from utils.data import GPSHandler 
from utils.data import ImuHandler 
from sensor_msgs.msg import Imu 
from sensor_msgs.msg import NavSatFix 
from std_msgs.msg import String

class DataHandler:
	"""
    This class handles how raspy read the IMU data from APM
    and send it to serial
    """
	def __init__(self):
		"""
        __init__ method of DataHandler class

        :return: None
        """
		self.writer = ch()
		self.fetch_flag = True
		self.toggle = True
		self.curr_alt = 0
		self.curr_dis = 0
		self.pub = rospy.Publisher("/aurora/senddata", String, queue_size=50)
	
	def __callback(self, imu):
		"""
        callback function, it received data from ImuHandler, then
        write it to serial port

		:Imu: Imu, mavros Imu Object
        :return: None
        """
		ih = ImuHandler(imu)
		rospy.loginfo(ih.get_acceleration())
		self.pub.publish(ih.get_acceleration())
		print(ih.get_acceleration())
		
	def start(self):
		"""
        start the data node program, this method never stop until
        the user terminate the program

        :return: None
        """
		rospy.Subscriber("/mavros/imu/data", Imu, self.__callback)
		rospy.spin()

def start():
	"""
	init ros node, and start the program

    :return: None
    """
	rospy.init_node('data_node', anonymous=True)
	dh = DataHandler()
	dh.start()

if __name__ == '__main__':
	start()
