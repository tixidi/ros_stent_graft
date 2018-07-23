#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np

currMsrTransformList = np.asarray([]) 
iiwa0_currMsrTransform = []
def callback_iiwa0_msrTransform(data):
	global currMsrTransformList, iiwa0_currMsrTransform
	print data.data
	iiwa0_currMsrTransform = np.asarray(list(data.data))
	if (currMsrTransformList.size ==0):
		currMsrTransformList = iiwa0_currMsrTransform
	else:
		currMsrTransformList = np.vstack((currMsrTransformList,iiwa0_currMsrTransform))

	np.savetxt('currMsrTransformList.txt',currMsrTransformList)

rospy.init_node('iiwa0_msrTransform_subscriber', anonymous=True)
rospy.Subscriber("iiwa0_msrTransform", Float64MultiArray, callback_iiwa0_msrTransform)
while (1):
	print iiwa0_currMsrTransform
	
