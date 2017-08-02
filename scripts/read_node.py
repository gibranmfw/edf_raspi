#! /usr/bin/python

from communication import CommunicationHandler as ch
import rospy

def start():
	rospy.init_node('read_node', anonymous=True)
	reader = ch()
	while(True):
		try:
			input = reader.read()
			print(input)
		except KeyboardInterrupt:
			reader.close()
			break

if __name__ == "__main__":
	start()
