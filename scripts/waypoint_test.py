#! /usr/bin/env python

import rospy
from waypoint import WaypointHandler

def waypoint_test():
	wh = WaypointHandler()
	wh.clearwplist()
	wh.clear_client()
	#wh.addwaypoint(3,31,31)
	#wh.addwaypoint(1,2,3)
	#wh.addwaypoint(1,2,2)
	#wh.addwaypoint(1,0,0)
	#print(wh.sendwplist())
	
def start():
	rospy.init_node("test_waypoint", anonymous=True)
	waypoint_test()


if __name__ == "__main__":
	start()
