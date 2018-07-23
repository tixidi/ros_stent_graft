#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64

count = 0;
def callback_count(data):
	global count
	print data.data
	if data.data == count:
		count = count+1;
		pub_count.publish(data=count)
		rate = rospy.Rate(30) # 1hz
		#rate.sleep()
		
		print count
		
rospy.init_node('test_talker', anonymous=True)
pub_count = rospy.Publisher('count', Int64, queue_size=1)
#rospy.Subscriber("count", Int64, callback_count,queue_size=1)







test = 0;
while(1):
	test = test+1;
	#if count ==0:
	count = count +1
	print count
	pub_count.publish(data=count)
	rate = rospy.Rate(1000) # 1hz
	
	rate.sleep()
	
	
