"""
This file contains how raspy control
waypoints
"""

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
	"""
    This class handle how to control waypoints
    of APM
    """
	def __init__(self):
		"""
        __init__ method of WaypointHandler class

        :return: None
        """
		self.wp_list = WaypointList()

	def sendwplist(self):
		"""
        send new list of waypoints to apm

        :return: boolean, true if success and false if failed
        """
		clear_flag = self.clear_client()
		print(clear_flag)
		push_flag = self.push_wp()
		print(push_flag)
		set_curr_flag = self.set_current()
		print(set_curr_flag)
		return clear_flag and push_flag and set_curr_flag

	def set_current_client(self, wp_set_curr):
		"""
        set current waypoint client

		:wp_set_curr: WaypointSetCurrent, mavros WaypointSetCurrent object
        :return: boolean, true if success and false if failed
        """
		try:
			resp = rospy.ServiceProxy("/mavros/mission/set_current", wp_set_curr)
			flag = resp.call().success
			return flag 
		except rospy.ServiceException, e:
			print("Service call failed: %s" %e)
			return False

	def set_current(self):
		"""
        set current waypoint

        :return: boolean, true if success and false if failed
        """
		wp_set_curr = WaypointSetCurrent()
		wp_set_curr.wp_seq = 1
		return self.set_current_client(wp_set_curr)

	def clear_client(self):
		"""
        clear waypoint client

        :return: boolean, true if success and false if failed
        """
		try:
			response = rospy.ServiceProxy("mavros/mission/clear", WaypointClear)
			return response.call().success
		except rospy.ServiceException, e:
			print("Service call failed: %s" %e)
			return False
	
	def push_wp(self):
		"""
        push waypoint to APM

        :return: boolean, true if success and false if failed
        """
		wp_push = WaypointPush()
		wp_push.waypoints = self.wp_list.waypoints
		return self.push_client(wp_push)	
	
	def push_client(self, wp_push):
		"""
        push waypoint client

		:wp_push: WaypointPush, mavros WaypointPush object
        :return: boolean, true if success and false if failed
        """
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
		"""
        clear waypoint list

        :return: None
        """
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
		"""
        add waypoint to waypoint list

		:lat: double, latitude
		:long: double, longitude
		:alt: double, altitude
        :return: None
        """
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

