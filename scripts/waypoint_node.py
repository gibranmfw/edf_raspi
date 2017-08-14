#! /usr/bin/env python

import rospy
from communication import CommunicationHandler as ch
from waypoint import WaypointHandler
from std_msgs.msg import String

class WaypointNode:
	def __init__(self):
		self.reader = ch()
		self.wh = WaypointHandler()

	def __callback(self, data):
		if(type(data) == String):
			if(len(data.data) > 2):
				waypoint = data.data.split(" ")
				print(waypoint)
				for index in range(len(waypoint)):
					if "@" in waypoint[index]:
						lat, lon, alt = waypoint[index].split("@")
						self.wh.addwaypoint(float(lat.replace(",", ".")), float(lon.replace(",", ".")), float(alt.replace(",", ".")))
				print(self.wh.sendwplist())
	
	def start(self):
		rospy.Subscriber("/aurora/gcsdata", String, self.__callback)
		rospy.spin()

def start():
	rospy.init_node('waypoint_node', anonymous=True)
	wn = WaypointNode()
	wn.start()
					
if __name__ == "__main__":
	start()

