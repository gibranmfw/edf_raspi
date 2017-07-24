#! /usr/bin/env python
import rospy
import message_filters
from mavros_msgs.msg import Waypoint
from mavros_msgs.msg import CommandCode
from mavros_msgs.msg import WaypointList
from mavros_msgs.srv import WaypointPush
from mavros_msgs.srv import WaypointPull
from mavros_msgs.srv import WaypointSetCurrent

def start():
	rospy.init_node("waypoint", anonymous=True)
	wp = Waypoint()
	srv = WaypointPush()
	print(srv)
	#ngambil data waypoint dari gcs
	#set_waypoint
	#srv.request.waypoints.push_back(wp)
	
def set_waypoint(wp, x_lat, y_long, z_alt, current):
	wp.frame = Waypoint.FRAME_GLOBAL
	wp.command = CommandCode.NAV_WAYPOINT
	wp.is_current = current
	wp.autocontinue = true
	wp.x_lat = x_lat
	wp.y_long = y_long
	wp.z_alt = x_alt

if __name__ == "__main__":
	start()
