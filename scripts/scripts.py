#! /usr/bin/env python
import rospy
import message_filters
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

class GPSData:
	def __init__(self, data):
		self.data = data
	
	def seeData(self):
		print(self.data.latitude)

class ImuData:
	def __init__(self, data):
		self.data = data

	def seeData(self):
		print("{}".format(self.data.orientation.x))
		print(self.data.angular_velocity.x)
		print(self.data.linear_acceleration.x)

def start():
	rospy.init_node('scripts', anonymous=True)
	imu_sub = message_filters.Subscriber("/mavros/imu/data", Imu)
	gps_sub = message_filters.Subscriber("/mavros/global_position/raw/fix", NavSatFix)
	ts = message_filters.ApproximateTimeSynchronizer([imu_sub, gps_sub], 10, 1)
	ts.registerCallback(callback)
	rospy.spin()

def callback(imu, gps):
	id = ImuData(imu)
	gd = GPSData(gps)
	id.seeData()
	gd.seeData()

if __name__ == '__main__':
	start()
