import rospy
from mavros_msgs.msg import CommandCode
from mavros_msgs.msg import Waypoint
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import WaypointList

def set_waypoint():
	pose = PoseStamped()
	pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
	rospy.init_node('pubwp', anonymous=True)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pose.pose.position.x = 50
		pose.pose.position.y = 20
		pose.pose.position.z = 20
		pub.publish(pose)
		rate.sleep()

def nav_waypoint():
	wp = Waypoint()
	wplist = WaypointList()
	pub = rospy.Publisher('/mavros/mission/waypoints', WaypointList, queue_size=10)
	rospy.init_node('pubwp', anonymous=True)
	rate = rospy.Rate(10)
	wp.frame = Waypoint.FRAME_GLOBAL
	wp.command = CommandCode.NAV_WAYPOINT
	wp.is_current = False
	wp.autocontinue = False
	wp.x_lat = 20
	wp.y_long = 0
	wp.z_alt = 20
	wplist.waypoints.append(wp)
	while not rospy.is_shutdown():
		pub.publish(wplist)
		rate.sleep()


if __name__ == '__main__':
	try:
		nav_waypoint()
	except rospy.ROSInterruptException:
		pass
