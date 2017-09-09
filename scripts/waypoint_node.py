"""
This file contains definition of
ros node to manage Waypoints
"""

#! /usr/bin/env python

import rospy
from utils.communication import CommunicationHandler as ch
from utils.waypoint import WaypointHandler
from std_msgs.msg import String

class WaypointNode:
	"""
    This class handles how raspy manage APM
    waypoints
    """
	def __init__(self):
		"""
        __init__ method of WaypointNode class

        :return: None
        """
		self.reader = ch()
		self.wh = WaypointHandler()

	def __callback(self, data):
		"""
        callback function of WaypointNodes class
        it receive waypoints data in string, the program
        process the string into a readable waypoint data
        
        format of the string :
        latitude1@longitude1@altitude1 latitude2@longitude2@altitude2 latitude3@longitude3@altitude3 ...
		
		:param: data (String), string of waypoints data from gcs
        :return: None
        """
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
		"""
        start the waypoint node program, this method never stop until
        the user terminate the program

        :return: None
        """
		rospy.Subscriber("/aurora/gcsdata", String, self.__callback)
		rospy.spin()

def start():
	"""
	init ros node, and start the program

    :return: None
    """
	rospy.init_node('waypoint_node', anonymous=True)
	wn = WaypointNode()
	wn.start()
					
if __name__ == "__main__":
	start()

