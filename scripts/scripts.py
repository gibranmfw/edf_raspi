#! /usr/bin/env python
import rospy
import message_filters
import servo
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

class ESCHandler:
	def __init__(self):
		self.esc = servo.servo(2)
	def esc_on():
		self.esc.move_max()
	def esc_off():
		self.esc.move_min()

class SafetyHandler:
	def __init__(self, esc):
		self.off = False
		self.para = servo.servo(0)
		self.esc = esc
	def emergency_on():
		self.off = True
	def check_emergency():
		if(self.off):
			self.para.move_min()
			self.esc.esc_off()

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
