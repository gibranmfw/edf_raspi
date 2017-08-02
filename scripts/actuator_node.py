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

class ActuatorHandler:
	def __init__(self):
		self.imu_sub = message_filters.Subscriber("/mavros/imu/data", Imu)
		self.gps_sub = message_filters.Subscriber("/mavros/global_position/raw/fix", NavSatFix)
		self.sh = SafetyHandler()
		self.esc = ServoHandler(2)
		self.reader = ch()
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
		else:
			data = gh.get_data_w_distance(self.slat, self.slong)
			temp = data.split()
			self.curr_alt = float(temp[2])
			self.curr_dis = float(temp[3])
			self.sh.check_range(self.curr_dis)
			self.sh.check_range(self.curr_alt)

	def start(self):
		ts = message_filters.ApproximateTimeSynchronizer([self.imu_sub, self.gps_sub], 10, 1)
		ts.registerCallback(self.__callback)
		#rospy.spin()
		while(True):
			try:
				data = self.reader.read()
				if(data):
					if(data == "0\n"):
						print("motor off")
						self.esc.move_min()
					elif(data == "1\n"):
						print("motor on")
						self.esc.move_max()
					elif(data == "2\n"):
						print("parachute off")
						self.sh.off()
					elif(data == "3\n"):
						print("parachute on")
						self.sh.on()
				self.sh.check_emergency()
			except KeyboardInterrupt:
				self.reader.close()
				break

def start():
	rospy.init_node('actuator_node', anonymous=True)
	ah = ActuatorHandler()
	ah.start()

if __name__ == '__main__':
	start()
