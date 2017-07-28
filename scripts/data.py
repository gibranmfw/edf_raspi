#! /usr/bin/env python
import rospy
import math
import message_filters
from safety import SafetyHandler
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

class Formula:
	
	@staticmethod
	def deg_to_rad(degrees):
		return degrees * math.pi / 180.0
	
	#return distance between 2 coordinate in km
	@staticmethod
	def distance_2_coor(lat1, long1, lat2, long2):
		earthrad = 6371 #in km
		dlat = Formula.deg_to_rad(lat2 - lat1)
		dlong = Formula.deg_to_rad(long2 - long1)
		
		lat1 = Formula.deg_to_rad(lat1)
		lat2 = Formula.deg_to_rad(lat2)

		a = math.sin(dlat / 2.0) * math.sin(dlat / 2.0) + math.sin(dlong / 2.0) * math.sin(dlong / 2.0) * math.cos(lat1) * math.cos(lat2)
		c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
		return earthrad * c

class GPSHandler:
	def __init__(self, data):
		self.data = data

	def get_data(self):
		return "{} {} {}".format(self.data.latitude, self.data.longitude, self.data.altitude)

	def get_data_raw(self):
		return (self.data.latitude, self.data.longitude, self.data.altitude)

	def get_data_w_distance(self, slat, slong):
		clat = self.data.latitude
		clong = self.data.longitude
		calt = self.data.altitude
		distance = Formula.distance_2_coor(slat, slong, clat, clong) * 1000 #in meter
		return "{} {} {} {}".format(clat, clong, calt, distance)

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
	def __init__(self):#(self, start_lat, start_long):
		self.imu_sub = message_filters.Subscriber("/mavros/imu/data", Imu)
		self.gps_sub = message_filters.Subscriber("/mavros/global_positiroslaunch mavros apm.launch fcu_url:=///dev/ttyACM0:115200on/raw/fix", NavSatFix)
		self.sh = SafetyHandler()
		self.fetch_flag = True
		#self.slat = start_lat
		#self.slong = start_long

	def __callback(self, imu, gps):
		ih = ImuHandler(imu)
		gh = GPSHandler(gps)
		if(self.fetch_flag):
			self.slat, self.slong, alt  = gh.get_data_raw()
			self.fetch_flag = False
			print("{} {} {}".format(self.slat, self.slong, alt))
		else: 
			print(gh.get_data_w_distance(self.slat, self.slong))
		print(ih.get_acceleration())

	def start(self):
		ts = message_filters.ApproximateTimeSynchronizer([self.imu_sub, self.gps_sub], 10, 1)
		ts.registerCallback(self.__callback)
		rospy.spin()

