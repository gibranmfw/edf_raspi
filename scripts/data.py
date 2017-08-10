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

	def get_latlong(self):
		return (self.data.latitude, self.data.longitude)

	def get_data(self):
		return "{} {} {}".format(self.data.latitude, self.data.longitude, self.data.altitude)

	def get_data_raw(self):
		return (self.data.latitude, self.data.longitude, self.data.altitude)

	def get_data_w_dis(self, slat, slong):
		clat = self.data.latitude
		clong = self.data.longitude
		distance = Formula.distance_2_coor(slat, slong, clat, clong) * 1000
		return "{} {} {}".format(clat, clong, distance)

	def get_data_w_disalt(self, slat, slong):
		clat = self.data.latitude
		clong = self.data.longitude
		calt = self.data.altitude
		distance = Formula.distance_2_coor(slat, slong, clat, clong) * 1000 #in meter
		return "{} {} {} {}".format(clat, clong, distance, calt)

class ImuHandler:
	def __init__(self, data):
		self.data = data

	def get_orientation(self):
		return "{} {} {}".format(self.data.orientation.x, self.data.orientation.y, self.data.orientation.z)

	def get_angularv(self):
		return "{} {} {}".format(self.data.angular_velocity.x, self.data.angular_velocity.y, self.data.angular_velocity.z)

	def get_acceleration(self):
		return "{} {} {}".format(self.data.linear_acceleration.x, self.data.linear_acceleration.y, self.data.linear_acceleration.z)
