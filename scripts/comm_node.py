"""
This file contains definition of
ros node to make raspy read serial port
"""

#! /usr/bin/python
import rospy
from utils.communication import CommunicationHandler
from std_msgs.msg import String

ch = CommunicationHandler()

def start():
	"""
	init ros node, and start the program
	the program read input from serial port (xbee), 
	then publish it in /aurora/gcsdata

    :return: None
    """
	rospy.init_node("comm_node", anonymous=True)
	pub = rospy.Publisher("/aurora/gcsdata", String, queue_size=50)
	rospy.Subscriber("/aurora/senddata", String, callback)
	while(True):
		try:
			data = ch.read()
			rospy.loginfo(data)
			pub.publish(data)
		except KeyboardInterrupt:
			break

def callback(data):
	"""
	callback function of senddata topic,
	send attitude data from rocket to gcs with
	xbee

    :return: None
    """
	sdata = data.data + "\n"
	ch.write(sdata)

if __name__ == "__main__":
	start()
