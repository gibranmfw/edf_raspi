#! /usr/bin/env python

import rospy
from communication import CommunicationHandler as ch
from waypoint import WaypointHandler

class WaypointNode:
	def __init__(self):
		self.reader = ch()
		self.wh = WaypointHandler()
	
	def start():
		while(True):
			try:
				data = self.reader.read()
				if(len(data) > 1):
					waypoint = data.split(" ")
					for(item in waypoint):
						
					
