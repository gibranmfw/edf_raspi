#! /usr/bin/env python
import rospy
import message_filters
from communication import CommunicationHandler as ch
from safety import SafetyHandler
from servo import ServoHandler
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from data import GPSHandler
from data import ImuHandler
from std_msgs.msg import Float64
from std_msgs.msg import String

class ActuatorHandler:
	def __init__(self):
		#self.imu_sub = message_filters.Subscriber("/mavros/imu/data", Imu)
		#self.gps_sub = message_filters.Subscriber("/mavros/global_position/raw/fix", NavSatFix)
		#self.rel_alt = message_filters.Subscriber("/mavros/global_position/rel_alt", Float64)
		self.sh = SafetyHandler()
		self.esc = ServoHandler(2, 7)
		self.sh = SafetyHandler(self.esc)
		self.comm = ch()
		self.fetch_flag = True
		self.toggle = True
		self.slat = 0
		self.slong = 0
	
	def set_alt(self, alt):
		self.curr_alt = alt

	def toggle_flag(self):
		self.fetch_flag = True

	def fetch_data(self, gh):
		if(self.fetch_flag):
			self.slat, self.slong = gh.get_latlong()
			self.fetch_flag = False
		else:
			data = gh.get_data_w_dis(self.slat, self.slong)
			temp = data.split(" ")
			self.curr_dis = float(temp[2])
			#self.sh.check_range(self.curr_dis)
			#self.sh.check_range(self.curr_alt)
			data = "{} {}".format(data, self.curr_alt)
			print(data)
			self.comm.write(data + "\n")

	def __callback(self, data):
		if(type(data) == Float64):
			self.set_alt(data.data)
		elif(type(data) == String):
			if(data.data == "0\n"):
				print("motor off")
				self.esc.move1_min()
			elif(data.data == "1\n"):
				print("motor on")
				self.esc.move1_max()
			elif(data.data == "2\n"):
				print("toggle parachute")
				self.sh.emergency()
			elif(data.data == "3\n"):
				print("fetching home position")
				self.toggle_flag()
		else:
			gh = GPSHandler(data)
			self.fetch_data(gh)

	def start(self):
		#ts = message_filters.ApproximateTimeSynchronizer([self.gps_sub, self.rel_alt], 10, 0.5)
		#ts.registerCallback(self.__callback)
		#rospy.spin()
		rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.__callback)
		rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self.__callback)
		rospy.Subscriber("/aurora/gcsdata", String, self.__callback)
		rospy.spin()

def start():
	rospy.init_node('actuator_node', anonymous=True)
	ah = ActuatorHandler()
	ah.start()

if __name__ == '__main__':
	start()
