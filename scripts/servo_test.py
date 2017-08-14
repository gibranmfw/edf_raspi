import os
import sys
#import math
#import socket
import rospy
import time
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode

#servo_channel=2
parachute_channel=5
exec_time=1 #exc time in secs

def talker():
    pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=10)
    rospy.init_node('custom_talker', anonymous=True)
    r = rospy.Rate(10) #10hz
    msg = OverrideRCIn()
    start = time.time()
    flag=True #time flag  
    msg.channels[parachute_channel]=2000 #Desired PWM value
    #msg.channels[servo_channel]=2000
    #pub.publish(msg)
    #rospy.loginfo(msg)
    #r.sleep()
    #change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    while not rospy.is_shutdown() and flag:
        sample_time=time.time()
        if ((sample_time - start) > exec_time):
        	flag=False
        	rospy.loginfo(msg)
        	pub.publish(msg)
        	r.sleep()
		#change_mode(custom_mode='auto')

if __name__ == '__main__':
    rospy.wait_for_service('/mavros/set_mode')
    change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    resp1 = change_mode(custom_mode="manual")
    print (resp1)
        
    if "True" in str(resp1):
            try:
		#change_mode(custom_mode="auto")
                talker()
		change_mode(custom_mode="auto")
    
            except rospy.ROSInterruptException: pass
