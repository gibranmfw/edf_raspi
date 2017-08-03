#! /usr/bin/env python

import rospy
from communication import CommunicationHandler as ch
from waypoint import WaypointHandler

class WaypointNode:
	def __init__(self):
		self.reader = ch()
		self.wh = WaypointHandler()
	
	def start(self):
		while(True):
			try:
				data = self.reader.read()
				if(len(data) > 1):
					waypoint = data.split(" ")
					for index in range(len(waypoint)):
						lat, lon, alt = waypoint[index].split(",")
						self.wh.addwaypoint(lat, lon, alt)
					self.wh.sendwplist()
			except KeyboardInterrupt:
				self.reader.close()
				break

def start():
	rospy.init_node('waypoint_node', anonymous=True)
	wn = WaypointNode()
	wn.start()
					
if __name__ == "__main__":
	start()
