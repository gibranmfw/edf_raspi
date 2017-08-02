#! /usr/bin/env python
import rospy 
import message_filters 
from communication import CommunicationHandler as ch 
from sensor_msgs.msg import Imu 
from sensor_msgs.msg import NavSatFix 
from data import GPSHandler 
from data import ImuHandler 

class DataHandler:
	def __init__(self):
		self.imu_sub = message_filters.Subscriber("/mavros/imu/data", Imu)
		self.gps_sub = message_filters.Subscriber("/mavros/global_position/raw/fix", NavSatFix)
		self.writer = ch()
		self.fetch_flag = True
		self.toggle = True
		self.curr_alt = 0
		self.curr_dis = 0
	
	def __callback(self, imu, gps):
		ih = ImuHandler(imu)
		gh = GPSHandler(gps)
		if(self.fetch_flag):
			self.slat, self.slong, alt = gh.get_data_raw()
			self.fetch_flag = False
			data = "{} {} {}".format(self.slat, self.slong, alt)
		else:
			data = gh.get_data_w_distance(self.slat, self.slong)
		data = "{} {}\n".format(data, ih.get_acceleration())
		self.writer.write(data)
		print(data)
		
	def start(self):
		ts = message_filters.ApproximateTimeSynchronizer([self.imu_sub, self.gps_sub], 1, 1)
		ts.registerCallback(self.__callback)
		rospy.spin()

def start():
	rospy.init_node('data_node', anonymous=True)
	dh = DataHandler()
	dh.start()

if __name__ == '__main__':
	start()