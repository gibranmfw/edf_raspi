#! /usr/bin/python

import rospy
from communication import CommunicationHandler
from std_msgs.msg import String

def start():
	rospy.init_node("comm_node", anonymous=True)
	pub = rospy.Publisher("/aurora/gcsdata", String, queue_size=50)
	ch = CommunicationHandler()
	while(True):
		try:
			data = ch.read()
			rospy.loginfo(data)
			pub.publish(data)
		except KeyboardInterrupt:
			ch.close()
			break


if __name__ == "__main__":
	start()
