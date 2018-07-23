#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray, Bool, Int64
from sensor_msgs.msg import JointState
import numpy as np
import time

iiwa0_reached = False
exotica_complete = False
flag_sent = False
count = -1;

iiwa0_destJoints = np.asarray([]);
traj_received = np.asarray([]);

def callback_iiwa0_reached(data):
	global iiwa0_reached
	#print data.data
	iiwa0_reached = data.data

def callback_exotica_complete(data):
	global exotica_complete, flag_sent
	#print data.data
	if (exotica_complete==True):
		flag_sent = True

def callback_exotica_count(data):
	global count
	count = data.data
	
	
def callback_iiwa0_destJoints(data):
	global iiwa0_destJoints, traj_received
	tmp = np.zeros(shape=(1,7))
	for i in range(7):
		tmp[0,i] = data.position[i]
		
	iiwa0_destJoints = tmp
	if traj_received.size == 0:
		traj_received = iiwa0_destJoints
	else:
		traj_received = np.concatenate((traj_received,iiwa0_destJoints), axis = 0)
	np.savetxt('/home/charlie/Documents/workspace/ros_ws/traj_received.txt',traj_received )
	
def talker():
    global iiwa0_reached, exotica_complete, flag_sent, count, iiwa0_destJoints

    pub = rospy.Publisher('iiwa0_desiredEEInRob', Float64MultiArray, queue_size=10)
    pub1 = rospy.Publisher('iiwa0_desiredEEInRob_sent', Bool, queue_size=10)
    pub2 = rospy.Publisher('iiwa0_connected', Bool, queue_size=10)
    pub3 = rospy.Publisher('exotica_complete', Bool, queue_size=10)
    rospy.Subscriber("iiwa0_reached", Bool, callback_iiwa0_reached)
    rospy.Subscriber("exotica_complete", Bool, callback_exotica_complete)
    rospy.Subscriber("exotica_count", Int64, callback_exotica_count)
    rospy.Subscriber("iiwa0_destJoints", JointState, callback_iiwa0_destJoints)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(300) # 10hz

    data = np.loadtxt('/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/iiwa_0_2018-04-19-16-03-46_s.txt_smooth_quat_r002_quat')

    #i = -1
    while not rospy.is_shutdown():
        #if ((flag_sent)):
		#	i = i + 1;
		#	print i
		#	flag_sent = False
		#	pub3.publish(False)
		#	rate.sleep()
		#	time.sleep(1)
        #if (i > -1 and not exotica_complete):
		print count
		count += 1
		msg = Float64MultiArray()
			#msg.data = [-0.864252, -0.485083, 0.133277, 0.0865756, -0.000158412, -0.264671, -0.964339, 0.00806532, 0.503059, -0.833453, 0.228665, 0.15361]
			#msg.data = [-8.642523e-01, -4.850827e-01, 1.332769e-01, 8.657560e-02, -1.584119e-04, -2.646708e-01, -9.643388e-01, 8.065320e-03, 5.030586e-01, -8.334531e-01, 2.286655e-01, 1.536100e-01]
			#msg.data = [0.23763151,	-0.07399702,	0.02910623,	0.60840166,	0.57966368,	-0.4911481,	-0.22937084]
		msg.data = data[count,1:8]
		print msg.data
			#rospy.loginfo(msg)
		pub.publish(msg)
		rate.sleep()
		pub1.publish(True)
		rate.sleep()




if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
