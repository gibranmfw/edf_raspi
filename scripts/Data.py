#! /usr/bin/env python
import rospy
import message_filters
from safety import SafetyHandler
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

class GPSHandler:
	def __init__(self, data):
		self.data = data

	def get_data(self):
		return "{} {} {}".format(self.data.latitude, self.data.longitude, self.data.altitude)

class ImuHandler:
	def __init__(self, data):
		self.data = data

	def get_orientation(self):
		raise NotImplementedError

	def get_angularv(self):
		raise NotImplementedError

	def get_acceleration(self):
		return "{} {} {}".format(self.data.linear_acceleration.x, self.data.linear_acceleration.y, self.data.linear_acceleration.z)

class DataHandler:
	def __init__(self):
		rospy.init_node("scripts", anonymous=True)
		self.imu_sub = message_filters.Subscriber("/mavros/imu/data", Imu)
		self.gps_sub = message_filters.Subscriber("/mavros/global_position/raw/fix", NavSatFix)
		self.sh = Safetyhandler()

	def __callback(self, imu, gps):
		ih = ImuHandler(imu)
		gh = GPSHandler(gps)
		print(ih.get_data())
		print(gh.get_acceleration())

	def start(self):
		ts = message_filters.ApproximateTimeSynchronizer([self.imu_sub, self.gps_sub], 10, 1)
		ts.registerCallback(self.__callback)
		rospy.spin()
