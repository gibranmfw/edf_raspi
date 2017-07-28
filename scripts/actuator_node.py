#! /usr/bin/env python
import rospy
import message_filters
from safety import SafetyHandler
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from data import GPSHandler
from data import ImuHandler

class ActuatorHandler:
	def __init__(self):
		self.imu_sub = message_filters.Subscriber("/mavros/imu/data", Imu)
		self.gps_sub = message_filters.Subscriber("/mavros/global_position/raw/fix", NavSatFix)
		self.sh = SafetyHandler()
		self.fetch_flag = True
		self.motor = MotorHandler(2) #testing
		self.toggle = True

	def __callback(self, imu, gps):
		ih = ImuHandler(imu)
		gh = GPSHandler(gps)
		if(self.fetch_flag):
			self.slat, self.slong, alt = gh.get_data_raw()
			self.fetch_flag = False
			print("{} {} {}".format(self.slat, self.slong, alt))
		else:
			print(gh.get_data_w_distance(self.slat, self.slong))
		print(ih.get_acceleration())

	def start(self):
		ts = message_filters.ApproximateTimeSynchronizer([self.imu_sub, self.gps_sub], 10, 1)
		ts.registerCallback(self.__callback)
		rospy.spin()

def start():
	rospy.init_node('actuator_node', anonymous=True)
	ah = ActuatorHandler()
	ah.start()

if __name__ == '__main__':
	start()
