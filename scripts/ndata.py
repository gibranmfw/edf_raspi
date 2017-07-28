#! /usr/bin/env python
import rospy
from data import DataHandler

def start():
	rospy.init_node('ndata', anonymous=True)
	dh = DataHandler()
	dh.start()
	
if __name__ == '__main__':
	start()
