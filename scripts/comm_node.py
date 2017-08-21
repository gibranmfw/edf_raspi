#! /usr/bin/python

import rospy
from communication import CommunicationHandler
from std_msgs.msg import String

ch = CommunicationHandler()

def start():
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
	sdata = data.data + "\n"
	ch.write(sdata)

if __name__ == "__main__":
	start()
