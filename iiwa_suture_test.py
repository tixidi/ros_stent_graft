#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int64

import pyexotica as exo
from pyexotica.publish_trajectory import *
import numpy as np
import matplotlib as mpl
import sys

import cv2
from numpy.linalg import inv
sys.path.append("./")
from bayes_opt import BayesianOptimization
from pyquaternion import Quaternion

from time import sleep
import math
import time

##########################
Tau = 0.05
length = 2

iiwa0_currMsrTransform = []	##homogeneous trasformation
iiwa1_currMsrTransform = []	##homogeneous trasformation

iiwa0_desiredEEInRob = []		##quaternion
iiwa1_desiredEEInRob = []		##quaternion

iiwa0_reached = False;
iiwa1_reached = False;

iiwa0_connected = False
iiwa1_connected = False

exotica_complete = False

#iiwas_desiredEEInRob = np.zeros(shape=(1,14))

iiwa0_default_desiredEEInRob = [1.67658125e-01,	3.91014254e-02,	1.58723434e-01,	5.57527601e-01,	7.51268653e-01,	-2.11285285e-01,	-2.83049313e-01]
iiwa1_default_desiredEEInRob = [-0.15402268, 0.04296034, 0.14749935, 0.46677196, 0.43085328, 0.44358363, 0.63223647]

iiwa0_default_currMsrTransform = [-0.18725492,  0.84542901, -0.50018916, -0.59726775, -0.97520191, -0.09886336,  0.19798807, -0.25512601, 0.11793113,  0.52485612,  0.84297739,  0.40726137];
iiwa1_default_currMsrTransform = [0.15776873, -0.14297075,  0.97707544,  0.37679855, -0.98729879, -0.04151556,  0.15334503, -0.49452684, 0.01863774, -0.98885802, -0.14770564,  0.43736135]

iiwa0_destJoints = [0.4836, -0.8033, -0.1767, 1.6785, -2.7219, -1.6706, 1.9363];
iiwa1_destJoints = [-1.2385, 0.7513, 0.1361, -1.3966, -1.7155, -1.3462, 0.1125];

iiwa0_desiredEEInRob_received = False;
iiwa1_desiredEEInRob_received = False;

iiwa0_currMsrTransform_received = False
iiwa1_currMsrTransform_received = False

iiwa0_currJoints = []
iiwa0_currJoints_received = False

iiwa1_currJoints = []
iiwa1_currJoints_received = False


rate_hz = 300

##########################

def callback_iiwa0_destJoints(data):
	global iiwa0_destJoints
	for i in range(7):
		if (len(iiwa0_destJoints) != 0 or iiwa0_destJoints[i] != data.position[i]):
			msg = JointState()
			msg.name = ["iiwa0 joint 1","iiwa0 joint 2","iiwa0 joint 3","iiwa0 joint 4","iiwa0 joint 5","iiwa0 joint 6","iiwa0 joint 7"]
			msg.position = iiwa0_destJoints
			pub_iiwa0_destJoints.publish(msg)
			rate = rospy.Rate(rate_hz) # 1hz
			rate.sleep()
			break;

def callback_iiwa1_destJoints(data):
	global iiwa1_destJoints
	for i in range(7):
		if (len(iiwa1_destJoints) != 0 or iiwa1_destJoints[i] != data.position[i]):
			msg = JointState()
			msg.name = ["iiwa1 joint 1","iiwa1 joint 2","iiwa1 joint 3","iiwa1 joint 4","iiwa1 joint 5","iiwa1 joint 6","iiwa1 joint 7"]
			msg.position = iiwa1_destJoints
			pub_iiwa1_destJoints.publish(msg)
			rate = rospy.Rate(rate_hz) # 1hz
			rate.sleep()
			break;

def callback_exotica_complete(data):
	global exotica_complete, iiwa0_currMsrTransform_received, iiwa1_currMsrTransform_received
	if ((iiwa0_currMsrTransform_received == False or iiwa1_currMsrTransform_received == False) and exotica_complete != data.data):
		exotica_complete = True
		pub_exotica_complete.publish(data = exotica_complete)
	else:
		exotica_complete = False
		pub_exotica_complete.publish(data = exotica_complete)



def callback_iiwa0_reached(data):
	global iiwa0_reached
	#print "iiwa0_reached "+str(data.data)
	iiwa0_reached = data.data

def callback_iiwa1_reached(data):
	global iiwa1_reached
	#print data.data
	iiwa1_reached = data.data

def callback_iiwa0_connected(data):
	global iiwa0_connected
	iiwa0_connected = data.data
	
def callback_iiwa1_connected(data):
	global iiwa1_connected
	iiwa1_connected = data.data

def callback_iiwa0_msrTransform(data):
	global iiwa0_currMsrTransform, iiwa0_currMsrTransform_received
	iiwa0_currMsrTransform = np.array(np.asarray(list(data.data)))
	if (iiwa0_currMsrTransform.size == 0):
		iiwa0_currMsrTransform_received = False
	else:
		iiwa0_currMsrTransform_received = True
	#if ( not np.array_equal(iiwa0_currMsrTransform, np.array(np.asarray(list(data.data)))) or iiwa0_currMsrTransform.size==0):
	#	iiwa0_currMsrTransform = np.array(np.asarray(list(data.data)))
	iiwa0_currMsrTransform_received = True

def callback_iiwa1_msrTransform(data):
	global iiwa1_currMsrTransform, iiwa1_currMsrTransform_received
	iiwa1_currMsrTransform = np.array(np.asarray(list(data.data)))
	if (iiwa1_currMsrTransform.size == 0):
		iiwa1_currMsrTransform_received = False
	else:
		iiwa1_currMsrTransform_received = True
	#if (not np.array_equal(iiwa1_currMsrTransform, np.array(np.asarray(list(data.data)))) or iiwa1_currMsrTransform.size==0):
	#	iiwa1_currMsrTransform = np.array(np.asarray(list(data.data)))
	iiwa1_currMsrTransform_received = True

def callback_iiwa0_desiredEEInRob(data):
	#print data.data
	global iiwa0_desiredEEInRob, iiwa0_desiredEEInRob_received
	tmp = np.array(np.asarray(list(data.data)))
	if (np.array_equal(tmp, iiwa0_desiredEEInRob) or tmp.size==0):
		iiwa0_desiredEEInRob_received = False
	elif (tmp.size!=0 and np.array(np.asarray(list(iiwa0_desiredEEInRob))).size==0):
		iiwa0_desiredEEInRob = tmp
		iiwa0_desiredEEInRob_received = True
	iiwa0_desiredEEInRob = np.array(np.asarray(list(data.data)))
	iiwa0_desiredEEInRob_received = True

def callback_iiwa1_desiredEEInRob(data):
	#print data.data
	global iiwa1_desiredEEInRob, iiwa1_desiredEEInRob_received
	tmp = np.array(np.asarray(list(data.data)))
	if (np.array_equal(tmp, iiwa1_desiredEEInRob) or tmp.size==0):
		iiwa1_desiredEEInRob_received = False
	elif (tmp.size!=0 and np.array(np.asarray(list(iiwa1_desiredEEInRob))).size==0):
		iiwa1_desiredEEInRob = tmp
		iiwa1_desiredEEInRob_received = True
	iiwa1_desiredEEInRob = np.array(np.asarray(list(data.data)))
	iiwa1_desiredEEInRob_received = True

def callback_iiwa0_currJoints(data):
	#print data.data
	global iiwa0_currJoints, iiwa0_currJoints_received
	for i in range(7):
		iiwa0_currJoints.append(data.position[i])
	iiwa0_currJoints_received = True

def callback_iiwa1_currJoints(data):
	#print data.data
	global iiwa1_currJoints, iiwa1_currJoints_received
	for i in range(7):
		iiwa1_currJoints.append(data.position[i])
	iiwa1_currJoints_received = True
			
#publisher 
pub_iiwa0_destJoints = rospy.Publisher('iiwa0_destJoints', JointState, queue_size=100,latch=True)
pub_iiwa1_destJoints = rospy.Publisher('iiwa1_destJoints', JointState, queue_size=100,latch=True)
pub_exotica_complete = rospy.Publisher('exotica_complete', Bool, queue_size=10,latch=True)
pub_exotica_count = rospy.Publisher('exotica_count', Int64, queue_size=10,latch=True)

					
def main():
	global iiwa0_reached
	global iiwa1_reached
	global iiwa0_connected
	global iiwa1_connected
	global iiwa0_desiredEEInRob
	global iiwa1_desiredEEInRob
	global iiwa0_currMsrTransform
	global iiwa1_currMsrTransform
	global iiwa0_desiredEEInRob_received
	global iiwa1_desiredEEInRob_received
	global exotica_complete
	global iiwa0_destJoints
	global iiwa1_destJoints
	global iiwa0_currMsrTransform_received
	global iiwa1_currMsrTransform_received
	global rate_hz
	global iiwa0_start_quat
	global iiwa0_currJoints
	global iiwa0_currJoints_received
	global iiwa1_currJoints
	global iiwa1_currJoints_received
	count = 0;

	#Initialise ros
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
	rospy.init_node('exotica_collision_detection', anonymous=True)
	rate = rospy.Rate(rate_hz) # 1hz
		
	#subscriber
	rospy.Subscriber("iiwa0_desiredEEInRob", Float64MultiArray, callback_iiwa0_desiredEEInRob)
	rospy.Subscriber("iiwa0_reached", Bool, callback_iiwa0_reached)
	rospy.Subscriber("iiwa0_connected", Bool, callback_iiwa0_connected)
	rospy.Subscriber("iiwa0_msrTransform", Float64MultiArray, callback_iiwa0_msrTransform)
	rospy.Subscriber("iiwa0_destJoints", JointState, callback_iiwa0_destJoints)
	#rospy.Subscriber("iiwa0_desiredEEInRob_sent", Bool, callback_iiwa0_desiredEEInRob_sent)
	rospy.Subscriber("iiwa1_desiredEEInRob", Float64MultiArray, callback_iiwa1_desiredEEInRob)
	rospy.Subscriber("iiwa1_reached", Bool, callback_iiwa1_reached)
	rospy.Subscriber("iiwa1_connected", Bool, callback_iiwa1_connected)
	rospy.Subscriber("iiwa1_msrTransform", Float64MultiArray, callback_iiwa1_msrTransform)
	rospy.Subscriber("iiwa1_destJoints", JointState, callback_iiwa1_destJoints)
	rospy.Subscriber("iiwa0_currJoints", JointState, callback_iiwa0_currJoints)
	rospy.Subscriber("iiwa1_currJoints", JointState, callback_iiwa1_currJoints)
	#rospy.Subscriber("iiwa1_desiredEEInRob_sent", Bool, callback_iiwa1_desiredEEInRob_sent)	
	#rospy.Subscriber("iiwa0_currMsrTransform_sent", Bool, callback_iiwa0_currMsrTransform_sent)	
	#rospy.Subscriber("iiwa1_currMsrTransform_sent", Bool, callback_iiwa1_currMsrTransform_sent)	
	#rospy.Subscriber("exotica_complete", Bool, callback_exotica_complete)	
    
	exotica_complete = True
	pub_exotica_complete.publish(exotica_complete)
	
	exotica_count = 0;
	pub_exotica_complete.publish(exotica_count)
	rate.sleep()
	
	
	#print iiwa0_currMsrTransform_sent
	#print iiwa1_currMsrTransform_sent
	
	##for test purpose
	#iiwa0_currMsrTransform_sent = True####################delete
	#iiwa1_currMsrTransform_sent = True
	#iiwa0_currMsrTransform_sent = True####################delete
	#iiwa1_reached = True;####################delete
	#iiwa1_desiredEEInRob_sent = True;
	
	kuka = 0;
	mode = 1;
	overall_traj = np.asarray([])
	###########	
	exo.Setup.initRos()
	########
	while(1):
		print exotica_complete,
		print " ",
		print iiwa0_connected,
		print " ",
		print iiwa0_reached,
		print " ",
		print iiwa0_desiredEEInRob_received,
		print " ",
		print iiwa0_currMsrTransform_received,
		print " ",
		print iiwa0_currJoints_received
		#pub_exotica_complete.publish(exotica_complete)
		pub_exotica_count.publish(exotica_count)
		pub_exotica_complete.publish(exotica_complete)
		rate.sleep()
		#if (exotica_complete and (iiwa0_connected or iiwa1_connected) and (iiwa0_reached or iiwa1_reached) and (iiwa0_desiredEEInRob_received or iiwa1_desiredEEInRob_received) and (iiwa0_currMsrTransform_received or iiwa1_currMsrTransform_received) and (iiwa0_desiredEEInRob.size!=0 or iiwa1_desiredEEInRob.size!=0) and (iiwa0_currJoints_received or iiwa1_currJoints_received)):
		if True:
			#iiwa0_currMsrTransform_received = False
			#iiwa1_currMsrTransform_received = False
			exotica_complete = False;
			pub_exotica_complete.publish(exotica_complete)
			#print str(iiwa0_connected)+" "+str(iiwa0_desiredEEInRob_sent) + " " +str(iiwa1_desiredEEInRob_sent)+ " " +str(iiwa0_currMsrTransform_sent)+ " " +str(iiwa1_currMsrTransform_sent)
			#if False:
			#print "iiwa0_connected " + str(iiwa0_connected)
			#print "iiwa1_connected " + str(iiwa1_connected)
			if (iiwa0_connected and iiwa0_desiredEEInRob_received):
				print "iiwa0 current transform"
				#print data_man_tool[0,12:24]
				print iiwa0_currMsrTransform
				#iiwa0_currMsrTransform_sent = False
				#pub_iiwa0_currMsrTransform_sent.publish(False)
				print ""
				print "iiwa0_desiredEEInRob"
				print iiwa0_desiredEEInRob
				write_traj(0, iiwa0_currMsrTransform, iiwa0_desiredEEInRob)
				#write_traj(0, iiwa0_default_currMsrTransform, iiwa0_desiredEEInRob);
				#write_traj(0, iiwa0_default_currMsrTransform, iiwa0_default_desiredEEInRob);


			if (mode == 0):
				print "iiwa0 current transform"
				print iiwa0_default_currMsrTransform
				write_traj(0, iiwa0_default_currMsrTransform, iiwa0_desiredEEInRob);


			#if (iiwa0_connected and iiwa0_desiredEEInRob_sent and not iiwa0_currMsrTransform_sent and mode == 1):
				#write_traj(0, iiwa0_default_currMsrTransform, iiwa0_desiredEEInRob);

			if (iiwa1_connected and iiwa1_desiredEEInRob_received):

				print "iiwa1 current transform"
				#print data_man_tool[0,24:36]
				print iiwa1_currMsrTransform
				#iiwa1_currMsrTransform_sent = False
				#pub_iiwa1_currMsrTransform_sent.publish(False)
				print ""
				print "iiwa1_desiredEEInRob"
				print iiwa1_desiredEEInRob
				write_traj(1, iiwa1_currMsrTransform, iiwa1_desiredEEInRob)

			if (mode == 1):
				#print "iiwa1 current transform"
				#print iiwa1_default_currMsrTransform
				write_traj(1, iiwa1_default_currMsrTransform, iiwa1_default_desiredEEInRob)

			#if (iiwa1_connected and iiwa1_desiredEEInRob_sent and not iiwa1_currMsrTransform_sent and mode == 1):
				#write_traj(1, iiwa1_default_currMsrTransform, iiwa1_desiredEEInRob);


			if (iiwa0_reached or iiwa1_reached):

				replace_aico_trajectory(iiwa0_currJoints, [-1.2385, 0.7513, 0.1361, -1.3966, -1.7155, -1.3462, 0.1125])########################################################modify this
				solver=exo.Setup.loadSolver('{stentgraft_sewing_planning}/resources/aico_trajectory.xml')
				problem = solver.getProblem()

				for t in range(0,problem.T):
					problem.setRho('Frame0',1e5,t)
					problem.setRho('Frame1',1e5,t)
					problem.setRho('JointLimit',1e4,t)

				solution = solver.solve()
				if not is_trajectory_valid(problem, solution):
					print 'Trajectory invalid'

				save_trajectory(solution)
				#print type(solution)


				if overall_traj.size==0:
					overall_traj = solution
				else:
					overall_traj = np.concatenate((overall_traj,solution),axis=0)


				i = 0;

				print "number of solution is "+str(np.size(solution,0))

				##sending trajectories
				while(i<np.size(solution,0)):
					iiwa0_destJoints = solution[i,0:7]
					iiwa1_destJoints = solution[i,7:14]

					msg0 = JointState()
					msg0.name = ["iiwa0 joint 1","iiwa0 joint 2","iiwa0 joint 3","iiwa0 joint 4","iiwa0 joint 5","iiwa0 joint 6","iiwa0 joint 7"]
					msg0.position = iiwa0_destJoints
					pub_iiwa0_destJoints.publish(msg0)
					rate.sleep()

					msg1 = JointState()
					msg1.name = ["iiwa1 joint 1","iiwa1 joint 2","iiwa1 joint 3","iiwa1 joint 4","iiwa1 joint 5","iiwa1 joint 6","iiwa1 joint 7"]
					msg1.position = iiwa1_destJoints
					pub_iiwa1_destJoints.publish(msg1)
					rate.sleep()

					print str(count) + ' '+ str(i) + " ",
					print iiwa0_destJoints

					#print "iiwa0_reached ",
					#print iiwa0_reached
					#print "iiwa1_reached ",
					#print iiwa1_reached
					#if (iiwa0_reached or iiwa1_reached):
						#i = i + 1;
						#iiwa0_reached = False
						#iiwa1_reached = False
					time.sleep(0.02)
					i = i + 1;
					count = count+1;


				exotica_complete = True
				pub_exotica_complete.publish(exotica_complete)
				exotica_count = exotica_count + 1
				pub_exotica_count.publish(exotica_count)
				rate.sleep()
				#exotica_complete = False
				#pub_exotica_complete.publish(exotica_complete)
				iiwa0_currMsrTransform = []
				iiwa1_currMsrTransform = []
				iiwa0_currMsrTransform_received = False
				iiwa1_currMsrTransform_received = False
				iiwa0_currJoints = []
				iiwa1_currJoints = []
				iiwa0_currJoints_received = False
				iiwa0_currJoints_received = False
				iiwa0_desiredEEInRob = np.asarray([])
				iiwa1_desiredEEInRob = np.asarray([])
				print "finished"
				np.savetxt('overall_traj.txt', overall_traj)

	############
	# iiwa0_start_quat = iiwa0_default_desiredEEInRob
	# iiwa0_start_joint_state = iiwa0_destJoints
	# iiwa1_start_joint_state = iiwa1_destJoints
	# overall_traj = np.asarray([])
	# while(1):
    #
	# 	if ((iiwa0_desiredEEInRob_received or iiwa1_desiredEEInRob_received)):
	# 		write_traj_quat(0, iiwa0_start_quat, iiwa0_desiredEEInRob)
	# 		iiwa0_start_quat = iiwa0_desiredEEInRob;
	# 		write_traj_quat(1, iiwa1_default_desiredEEInRob, iiwa1_default_desiredEEInRob)
	# 		iiwa1_start_quat = iiwa0_desiredEEInRob;
    #
	# 		replace_aico_trajectory(iiwa0_start_joint_state, iiwa1_start_joint_state)
	# 		solver=exo.Setup.loadSolver('{stentgraft_sewing_planning}/resources/aico_trajectory.xml')
	# 		problem = solver.getProblem()
    #
	# 		for t in range(0,problem.T):
	# 			problem.setRho('Frame0',1e5,t)
	# 			problem.setRho('Frame1',1e5,t)
	# 			problem.setRho('JointLimit',1e4,t)
    #
	# 		solution = solver.solve()
	# 		if not is_trajectory_valid(problem, solution):
	# 			print 'Trajectory invalid'
    #
	# 		save_trajectory(solution)
	# 		#print type(solution)
    #
	# 		#iiwa1_start_quat = iiwa1_default_desiredEEInRob
    #
	# 		print "solution length ",
	# 		print solution.shape[0]
	# 		print " exotica count ",
	# 		print exotica_count
    #
	# 		if (solution.shape[0]!=0):
	# 			if overall_traj.size == 0:
	# 				overall_traj = solution
	# 			else:
	# 				overall_traj = np.concatenate((overall_traj,solution),axis=0)
	# 			for i in range(solution.shape[0]):
	# 				iiwa0_destJoints = solution[i,0:7]
	# 				msg0 = JointState()
	# 				msg0.name = ["iiwa0 joint 1","iiwa0 joint 2","iiwa0 joint 3","iiwa0 joint 4","iiwa0 joint 5","iiwa0 joint 6","iiwa0 joint 7"]
	# 				msg0.position = iiwa0_destJoints
	# 				pub_iiwa0_destJoints.publish(msg0)
	# 				rate.sleep()
    #
	# 			iiwa0_start_joint_state = solution[solution.shape[0]-1,0:7]
	# 			iiwa1_start_joint_state = solution[solution.shape[0]-1,7:14]
    #
    #
	# 		np.savetxt('overall_traj.txt', overall_traj)
	# 		exotica_count = exotica_count + 1
 	# 		pub_exotica_count.publish(exotica_count)
	# 	pub_exotica_count.publish(exotica_count)
	############

def write_traj_quat(iiwaNo, iiwa_start_quat, iiwa_end_quat):
	iiwa_traj = np.zeros(shape=(2,8))
	iiwa_traj[0, 1] = iiwa_start_quat[0]
	iiwa_traj[0, 2] = iiwa_start_quat[1]
	iiwa_traj[0, 3] = iiwa_start_quat[2]
	iiwa_traj[0, 4] = iiwa_start_quat[3]
	iiwa_traj[0, 5] = iiwa_start_quat[4]
	iiwa_traj[0, 6] = iiwa_start_quat[5]
	iiwa_traj[0, 7] = iiwa_start_quat[6]
	
	iiwa_traj[1, 1] = iiwa_end_quat[0]
	iiwa_traj[1, 2] = iiwa_end_quat[1]
	iiwa_traj[1, 3] = iiwa_end_quat[2]
	iiwa_traj[1, 4] = iiwa_end_quat[3]
	iiwa_traj[1, 5] = iiwa_end_quat[4]
	iiwa_traj[1, 6] = iiwa_end_quat[5]
	iiwa_traj[1, 7] = iiwa_end_quat[6]

		#dt = 10.0 / (length - 1.0)
	dt = 10;
	for i in range(0, 2):
		iiwa_traj[i, 0] = i * dt

	np.set_printoptions(threshold='nan')

	tmp = '1\n' + str(length) + '\t8\n'
	iiwa_traj = np.array2string(iiwa_traj, separator='\t', max_line_width=np.inf)

	iiwa_traj = iiwa_traj.replace('[', '')
	iiwa_traj = iiwa_traj.replace(']', '')
	iiwa_traj = iiwa_traj.replace(' ', '')
	pack_path = exo.Tools.parsePath('{stentgraft_sewing_planning}/resources/')
	text_file = open(pack_path + 'iiwa_'+str(iiwaNo)+'.traj', "w")
	text_file.write(tmp + iiwa_traj)
	text_file.close()
	

def replace_aico_trajectory(iiwa0_start_joint_state, iiwa1_start_joint_state):

	i0 = iiwa0_start_joint_state
	i1 = iiwa1_start_joint_state
	#print "i0 " + str(i0[0])+' '+str(i0[1])+' '+str(i0[2])+' '+str(i0[3])+' '+str(i0[4])+' '+str(i0[5])+' '+str(i0[6])
	#print "i1 " + str(i1[0])+' '+str(i1[1])+' '+str(i1[2])+' '+str(i1[3])+' '+str(i1[4])+' '+str(i1[5])+' '+str(i1[6])
	pack_path = exo.Tools.parsePath('{stentgraft_sewing_planning}/resources/')
	aico_traj_file_read = open(pack_path + 'aico_trajectory.xml',"r")
	aico_traj_content = aico_traj_file_read.read()
	aico_traj_file_read.close()
	#print aico_traj_content
	tmp1 = aico_traj_content.split("<StartState>")
	tmp2 = aico_traj_content.split("</StartState>")
	#print tmp1[0]
	#print '---------------------------------------'
	#print tmp2[1]+tmp2[2]
	#print '---------------------------------------'
	aico_traj_content = tmp1[0]+"<StartState>"+str(i0[0])+' '+str(i0[1])+' '+str(i0[2])+' '+str(i0[3])+' '+str(i0[4])+' '+str(i0[5])+' '+str(i0[6])+' '+str(i1[0])+' '+str(i1[1])+' '+str(i1[2])+' '+str(i1[3])+' '+str(i1[4])+' '+str(i1[5])+' '+str(i1[6])+"</StartState>"+tmp2[1]+"</StartState>"+tmp2[2]
	#print aico_traj_content
	aico_traj_content_write = open(pack_path + 'aico_trajectory.xml',"w")
	aico_traj_content_write.write(aico_traj_content)	
	aico_traj_content_write.close()
 
	
def write_traj(iiwaNo, iiwa_start_trans, iiwa_end_quat):
	print "iiwa_end_quat "
	print iiwa_end_quat
	pack_path = exo.Tools.parsePath('{stentgraft_sewing_planning}/resources/')
	iHee = np.eye(4,4)
	for i in range(4):
		iHee[0,i] = iiwa_start_trans[i]
		iHee[1,i] = iiwa_start_trans[i+4]
		iHee[2,i] = iiwa_start_trans[i+8]	
	#print iHee
	if (iiwaNo == 0):
		cHi0 = np.loadtxt(pack_path + 'iiwa02CameraTransFile.txt')
		cHee0 = np.matmul(cHi0,iHee)
		#print cHee0
		#iiwa_start_quat = quaternion(cHee0[0:3,0:3])
		cHee = cHee0
	
	if (iiwaNo == 1):
		cHi1 = np.loadtxt(pack_path + 'iiwa12CameraTransFile.txt')
		cHee1 = np.matmul(cHi1,iHee)
		#iiwa_start_quat = quaternion(cHee1[0:3,0:3])
		cHee = cHee1

	u, s, vh = np.linalg.svd(cHee[0:3,0:3], full_matrices=True)
	cHee[0:3,0:3] = np.matmul(u, vh)

	iiwa_start_quat = Quaternion(matrix=cHee)
	
	
	print iiwaNo
	iiwa_traj = np.zeros(shape=(2,8))

	#iiwa_traj[0, 1] = iiwa_start_trans[3]
	#iiwa_traj[0, 2] = iiwa_start_trans[7]
	#iiwa_traj[0, 3] = iiwa_start_trans[11]
	iiwa_traj[0, 1] = cHee[0,3]
	iiwa_traj[0, 2] = cHee[1,3]
	iiwa_traj[0, 3] = cHee[2,3]
	iiwa_traj[0, 4] = iiwa_start_quat[1]
	iiwa_traj[0, 5] = iiwa_start_quat[2]
	iiwa_traj[0, 6] = iiwa_start_quat[3]
	iiwa_traj[0, 7] = iiwa_start_quat[0]
	
	iiwa_traj[1, 1] = iiwa_end_quat[0]
	iiwa_traj[1, 2] = iiwa_end_quat[1]
	iiwa_traj[1, 3] = iiwa_end_quat[2]
	iiwa_traj[1, 4] = iiwa_end_quat[3]
	iiwa_traj[1, 5] = iiwa_end_quat[4]
	iiwa_traj[1, 6] = iiwa_end_quat[5]
	iiwa_traj[1, 7] = iiwa_end_quat[6]



		#dt = 10.0 / (length - 1.0)
	dt = 10;
	for i in range(0, 2):
		iiwa_traj[i, 0] = i * dt

	np.set_printoptions(threshold='nan')

	tmp = '1\n' + str(length) + '\t8\n'
	iiwa_traj = np.array2string(iiwa_traj, separator='\t', max_line_width=np.inf)

	iiwa_traj = iiwa_traj.replace('[', '')
	iiwa_traj = iiwa_traj.replace(']', '')
	iiwa_traj = iiwa_traj.replace(' ', '')
	pack_path = exo.Tools.parsePath('{stentgraft_sewing_planning}/resources/')
	text_file = open(pack_path + 'iiwa_'+str(iiwaNo)+'.traj', "w")
	text_file.write(tmp + iiwa_traj)
	text_file.close()


	# print length, ' poses -> iiwa_0.traj, iiwa_1.traj'


def save_trajectory(solution):
	pack_path = exo.Tools.parsePath('{stentgraft_sewing_planning}')
	np.savetxt(pack_path+'/resources/traj_solution',solution)

def is_trajectory_valid(problem, solution):
	length = np.size(solution,0)
	for i in range(0,length):	
		problem.getScene().Update(solution[i], Tau*i)
		if not problem.getScene().isStateValid():
			print 'Way point ', i, '/', length, ' is in collision'
			problem.getScene().publishScene()
			return False
	return True

def plotEndEeffector(problem, traj):
	iiwa0_eff_traj = []
	iiwa0_target_traj = []
	iiwa1_eff_traj = []
	iiwa1_target_traj = []

	length = np.size(traj,0)
	for i in range(0,length):
		problem.getScene().Update(traj[i], Tau*i)
		iiwa0_eff_pose = np.append(problem.getScene().fk('iiwa_0_suture_driver','iiwa_0_base_link').getTranslation(), problem.getScene().fk('iiwa_0_suture_driver','iiwa_0_base_link').getQuaternion())
		iiwa0_eff_traj.append(iiwa0_eff_pose)
		iiwa1_eff_pose = np.append(problem.getScene().fk('iiwa_1_needle_driver','iiwa_1_base_link').getTranslation(),problem.getScene().fk('iiwa_1_needle_driver','iiwa_1_base_link').getQuaternion())
		iiwa1_eff_traj.append(iiwa1_eff_pose)

		iiwa0_target_pose = np.append(problem.getScene().fk('Target0','iiwa_0_base_link').getTranslation(), problem.getScene().fk('Target0','iiwa_0_base_link').getQuaternion())
		iiwa0_target_traj.append(iiwa0_target_pose)
		iiwa1_target_pose = np.append(problem.getScene().fk('Target1','iiwa_1_base_link').getTranslation(), problem.getScene().fk('Target1','iiwa_1_base_link').getQuaternion())
		iiwa1_target_traj.append(iiwa1_target_pose)

	iiwa0_eff_traj = np.vstack(iiwa0_eff_traj)
	iiwa1_eff_traj = np.vstack(iiwa1_eff_traj)
	iiwa0_target_traj = np.vstack(iiwa0_target_traj)
	iiwa1_target_traj = np.vstack(iiwa1_target_traj)

	mpl.rcParams['lines.linewidth'] = 3
	plt.subplot(2,3,1)
	plt.plot(iiwa0_eff_traj[:,0],'r')
	plt.plot(iiwa0_target_traj[:,0],'g')
	plt.title('iiwa 0: X')
	plt.subplot(2,3,2)
	plt.plot(iiwa0_eff_traj[:,1],'r')
	plt.plot(iiwa0_target_traj[:,1],'g')
	plt.title('iiwa 0: Y')
	plt.subplot(2,3,3)
	plt.plot(iiwa0_eff_traj[:,2],'r')
	plt.plot(iiwa0_target_traj[:,2],'g')
	plt.title('iiwa 0: Z')
	plt.subplot(2,3,4)
	plt.plot(iiwa1_eff_traj[:,0],'r')
	plt.plot(iiwa1_target_traj[:,0],'g')
	plt.title('iiwa 1: X')
	plt.subplot(2,3,5)
	plt.plot(iiwa1_eff_traj[:,1],'r')
	plt.plot(iiwa1_target_traj[:,1],'g')
	plt.title('iiwa 1: Y')
	plt.subplot(2,3,6)
	plt.plot(iiwa1_eff_traj[:,2],'r')
	plt.plot(iiwa1_target_traj[:,2],'g')
	plt.title('iiwa 1: Z')
	plt.show()
	
def plotSolution(solution):
	solution = solution/3.14*180
	for i in range(0,14):
		plt.subplot(2,7,i+1)
		plt.plot(solution[:,i])
		plt.title('joint '+str(i%7))
	plt.show()


if __name__ == '__main__':
    main()


    


