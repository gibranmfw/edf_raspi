"""
This file contains basic class to control
data from APM (Arduplane Module)
"""

#! /usr/bin/env python
import rospy
import math
import message_filters
from safety import SafetyHandler
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

class Formula:
	"""
    This class contains formulas that often used
    in the file
    """
	@staticmethod
	def deg_to_rad(degrees):
		"""
        convert degrees to radian

		:degrees: double, degrees to convert
        :return: double
        """
		return degrees * math.pi / 180.0
	
	@staticmethod
	def distance_2_coor(lat1, long1, lat2, long2):
		"""
        Calculate distance between 2 coordinate

		:lat1: double, starting latitude
		:long1: double, starting longitude
		:lat2: double, end latitude
		:long2: double, end longitude
        :return: double, distance between 2 coordinate in km
        """
		earthrad = 6371 #in km
		dlat = Formula.deg_to_rad(lat2 - lat1)
		dlong = Formula.deg_to_rad(long2 - long1)
		
		lat1 = Formula.deg_to_rad(lat1)
		lat2 = Formula.deg_to_rad(lat2)

		a = math.sin(dlat / 2.0) * math.sin(dlat / 2.0) + math.sin(dlong / 2.0) * math.sin(dlong / 2.0) * math.cos(lat1) * math.cos(lat2)
		c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
		return earthrad * c

class GPSHandler:
	"""
    This class handle how raspi get GPS data from APM
    """
	def __init__(self, data):
		"""
        __init__ method of GPSHandler class

		:data: NavSatFix, mavros gps class
        :return: None
        """
		self.data = data
	
	def get_latlong(self):
		"""
        get current coordinate in tuple

        :return: Tuple, latitude and longitude of current position
        """
		return (self.data.latitude, self.data.longitude)

	def get_data(self):
		"""
        get current coordinate and altitude in String

        :return: String, latitude, longitude, and altitude
        """
		return "{} {} {}".format(self.data.latitude, self.data.longitude, self.data.altitude)

	def get_data_raw(self):
		"""
        get current coordinate and altitude in Tuple

        :return: Tuple, latitude, longitude, and altitude
        """
		return (self.data.latitude, self.data.longitude, self.data.altitude)

	def get_data_w_dis(self, slat, slong):
		"""
        get current coordinate and distance in String

		:slat: double, starting latitude
		:slong: double, starting Longitude
        :return: String, latitude, longitude, and distance
        """
		clat = self.data.latitude
		clong = self.data.longitude
		distance = Formula.distance_2_coor(slat, slong, clat, clong) * 1000
		print(distance)
		return "{} {} {}".format(clat, clong, distance)

	def get_data_w_disalt(self, slat, slong):
		"""
        get current coordinate, altitude and distance in String

		:slat: double, starting latitude
		:slong: double, starting Longitude
        :return: String, latitude, longitude, distance, and altitude
        """
		clat = self.data.latitude
		clong = self.data.longitude
		calt = self.data.altitude
		distance = Formula.distance_2_coor(slat, slong, clat, clong) * 1000 #in meter
		return "{} {} {} {}".format(clat, clong, distance, calt)

class ImuHandler:
	"""
    This class handle how raspi get IMU data from APM
    """
	def __init__(self, data):
		"""
        __init__ method of ImuHandler class

		:data: Imu, mavros IMU class
        :return: None
        """
		self.data = data

	def get_orientation(self):
		"""
        get current x, y, and z-axis orientation

        :return: String, x-axis orientation, y-axis orientation, z-axis orientation
        """
		return "{} {} {}".format(self.data.orientation.x, self.data.orientation.y, self.data.orientation.z)

	def get_angularv(self):
		"""
        get current x, y, and z-axis angular velocity

        :return: String, x-axis angular velocity, y-axis angular velocity, z-axis angular velocity
        """
		return "{} {} {}".format(self.data.angular_velocity.x, self.data.angular_velocity.y, self.data.angular_velocity.z)

	def get_acceleration(self):
		"""
        get current x, y, and z-axis acceleration

        :return: String, x-axis acceleration, y-axis acceleration, z-axis acceleration
        """
		return "{} {} {}".format(self.data.linear_acceleration.x, self.data.linear_acceleration.y, self.data.linear_acceleration.z)
