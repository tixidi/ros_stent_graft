#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64

count = -1;
def callback_count(data):
	global count
	count = data.data
	
rospy.init_node('test_listener', anonymous=True)
rospy.Subscriber("count", Int64, callback_count,queue_size=1)

while(1):
	print count
	
