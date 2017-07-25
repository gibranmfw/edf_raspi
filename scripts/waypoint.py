#! /usr/bin/env python
import rospy
import message_filters
from mavros_msgs.msg import Waypoint
from mavros_msgs.msg import CommandCode
from mavros_msgs.msg import WaypointList
from mavros_msgs.srv import WaypointPush
from mavros_msgs.srv import WaypointClear
from mavros_msgs.srv import WaypointSetCurrent

class WaypointHandler:

	def __init__(self):
		self.wp_list = WaypointList()

	def sendwplist(self):
		clear_flag = self.clear_client()
		print(clear_flag)
		push_flag = self.push_wp()
		print(push_flag)
		set_curr_flag = self.set_current()
		print(set_curr_flag)
		return clear_flag and push_flag and set_curr_flag

	def set_current_client(self, wp_set_curr):
		try:
			resp = rospy.ServiceProxy("/mavros/mission/set_current", wp_set_curr)
			flag = resp.call().success
			return flag 
		except rospy.ServiceException, e:
			print("Service call failed: %s" %e)
			return False

	def set_current(self):
		wp_set_curr = WaypointSetCurrent()
		wp_set_curr.wp_seq = 1
		return self.set_current_client(wp_set_curr)

	def clear_client(self):
		try:
			response = rospy.ServiceProxy("mavros/mission/clear", WaypointClear)
			return response.call().success
		except rospy.ServiceException, e:
			print("Service call failed: %s" %e)
			return False
	
	def push_wp(self):
		wp_push = WaypointPush()
		wp_push.waypoints = self.wp_list.waypoints
		return self.push_client(wp_push)	
	
	def push_client(self, wp_push):
		try:
			resp = rospy.ServiceProxy("/mavros/mission/push", wp_push)
			flag = resp.call(self.wp_list.waypoints).success
			if flag:
				print("write mission success")
			else:
				print "write mission error"
			return flag
		except rospy.ServiceException, e:
			print("Service call failed: %s" % e)
			return False
	
	def clearwplist(self):
		defaultwp = Waypoint()
		defaultwp.frame = Waypoint.FRAME_GLOBAL
		defaultwp.command = CommandCode.NAV_WAYPOINT
		defaultwp.is_current = False
		defaultwp.autocontinue = True
		defaultwp.param1 = 0
		defaultwp.param2 = 0
		defaultwp.param3 = 0
		defaultwp.param4 = 0
		defaultwp.x_lat = 0
		defaultwp.y_long = 0
		defaultwp.z_alt = 0

		self.wp_list.waypoints[:] = []
		self.wp_list.waypoints.append(defaultwp)
		
	def addwaypoint(self, lat, long, alt):
		new_wp = Waypoint()
		new_wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
		new_wp.command = CommandCode.NAV_WAYPOINT
		new_wp.is_current = True
		new_wp.autocontinue = True
		new_wp.param1 = 0
		new_wp.param2 = 0
		new_wp.param3 = 0
		new_wp.param4 = 0
		new_wp.x_lat = lat
		new_wp.y_long = long
		new_wp.z_alt = alt

		self.wp_list.waypoints.append(new_wp)

