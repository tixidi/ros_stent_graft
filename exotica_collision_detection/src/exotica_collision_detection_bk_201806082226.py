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

iiwa0_destJoints = [];
iiwa1_destJoints = [];

iiwa0_desiredEEInRob_received = False;
iiwa1_desiredEEInRob_received = False;

iiwa0_currMsrTransform_received = False
iiwa1_currMsrTransform_received = False

##########################

def callback_iiwa0_destJoints(data):
	global iiwa0_destJoints
	for i in range(7):
		if (len(iiwa0_destJoints) != 0 or iiwa0_destJoints[i] != data.position[i]):
			msg = JointState()
			msg.name = ["iiwa0 joint 1","iiwa0 joint 2","iiwa0 joint 3","iiwa0 joint 4","iiwa0 joint 5","iiwa0 joint 6","iiwa0 joint 7"]
			msg.position = iiwa0_destJoints
			pub_iiwa0_destJoints.publish(msg)
			break;

def callback_iiwa1_destJoints(data):
	global iiwa1_destJoints
	for i in range(7):
		if (len(iiwa1_destJoints) != 0 or iiwa1_destJoints[i] != data.position[i]):
			msg = JointState()
			msg.name = ["iiwa1 joint 1","iiwa1 joint 2","iiwa1 joint 3","iiwa1 joint 4","iiwa1 joint 5","iiwa1 joint 6","iiwa1 joint 7"]
			msg.position = iiwa1_destJoints
			pub_iiwa1_destJoints.publish(msg)
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
	count = 0;

	#Initialise ros
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
	rospy.init_node('exotica_collision_detection', anonymous=True)
	rate = rospy.Rate(30) # 1hz
		
	#subscriber
	rospy.Subscriber("iiwa0_desiredEEInRob", Float64MultiArray, callback_iiwa0_desiredEEInRob)
	rospy.Subscriber("iiwa0_reached", Bool, callback_iiwa0_reached, tcp_nodelay=True)
	rospy.Subscriber("iiwa0_connected", Bool, callback_iiwa0_connected)
	rospy.Subscriber("iiwa0_msrTransform", Float64MultiArray, callback_iiwa0_msrTransform)
	rospy.Subscriber("iiwa0_destJoints", JointState, callback_iiwa0_destJoints, tcp_nodelay=True)
	#rospy.Subscriber("iiwa0_desiredEEInRob_sent", Bool, callback_iiwa0_desiredEEInRob_sent)
	rospy.Subscriber("iiwa1_desiredEEInRob", Float64MultiArray, callback_iiwa1_desiredEEInRob)
	rospy.Subscriber("iiwa1_reached", Bool, callback_iiwa1_reached, tcp_nodelay=True)
	rospy.Subscriber("iiwa1_connected", Bool, callback_iiwa1_connected)
	rospy.Subscriber("iiwa1_msrTransform", Float64MultiArray, callback_iiwa1_msrTransform)
	rospy.Subscriber("iiwa1_destJoints", JointState, callback_iiwa1_destJoints, tcp_nodelay=True)
	#rospy.Subscriber("iiwa1_desiredEEInRob_sent", Bool, callback_iiwa1_desiredEEInRob_sent)	
	#rospy.Subscriber("iiwa0_currMsrTransform_sent", Bool, callback_iiwa0_currMsrTransform_sent)	
	#rospy.Subscriber("iiwa1_currMsrTransform_sent", Bool, callback_iiwa1_currMsrTransform_sent)	
	#rospy.Subscriber("exotica_complete", Bool, callback_exotica_complete)	
    
	exotica_complete = True
	#pub_exotica_complete.publish(exotica_complete)
	
	exotica_count = 0;
	pub_exotica_complete.publish(exotica_count)
	
	
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
	###########	
	exo.Setup.initRos()
	while(1):
		print exotica_complete,
		print " ",
		print iiwa0_connected,
		print " ",
		print iiwa0_reached,
		print " ",
		print iiwa0_desiredEEInRob_received,
		print " ",
		print iiwa0_currMsrTransform_received
		#pub_exotica_complete.publish(exotica_complete)
		pub_exotica_count.publish(exotica_count)
		if (exotica_complete and (iiwa0_connected or iiwa1_connected) and (iiwa0_reached or iiwa1_reached) and (iiwa0_desiredEEInRob_received or iiwa1_desiredEEInRob_received) and (iiwa0_currMsrTransform_received or iiwa1_currMsrTransform_received)):
			#iiwa0_currMsrTransform_received = False
			#iiwa1_currMsrTransform_received = False
			exotica_complete = False;
			pub_exotica_complete.publish(exotica_complete)
			#print str(iiwa0_connected)+" "+str(iiwa0_desiredEEInRob_sent) + " " +str(iiwa1_desiredEEInRob_sent)+ " " +str(iiwa0_currMsrTransform_sent)+ " " +str(iiwa1_currMsrTransform_sent)
			#if False:
			print "iiwa0_connected " + str(iiwa0_connected)
			print "iiwa1_connected " + str(iiwa1_connected)
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
				print "iiwa1 current transform"
				print iiwa1_default_currMsrTransform
				write_traj(1, iiwa1_default_currMsrTransform, iiwa1_default_desiredEEInRob)

			#if (iiwa1_connected and iiwa1_desiredEEInRob_sent and not iiwa1_currMsrTransform_sent and mode == 1):
				#write_traj(1, iiwa1_default_currMsrTransform, iiwa1_desiredEEInRob);


			if (iiwa0_reached or iiwa1_reached):

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

					msg1 = JointState()
					msg1.name = ["iiwa1 joint 1","iiwa1 joint 2","iiwa1 joint 3","iiwa1 joint 4","iiwa1 joint 5","iiwa1 joint 6","iiwa1 joint 7"]
					msg1.position = iiwa1_destJoints
					pub_iiwa1_destJoints.publish(msg1)
						
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
				#pub_exotica_complete.publish(exotica_complete)
				exotica_count = exotica_count + 1
				pub_exotica_count.publish(exotica_count)
				#exotica_complete = False
				#pub_exotica_complete.publish(exotica_complete)
				iiwa0_currMsrTransform = []
				iiwa1_currMsrTransform = []
				iiwa0_currMsrTransform_received = False
				iiwa1_currMsrTransform_received = False
				iiwa0_desiredEEInRob = []
				iiwa1_desiredEEInRob = []
				print "finished"
				
			
			#pub_iiwa0_reached.publish(True)####################delete
			#np.savetext('iiwa0_received_desiredEEInRob.txt',iiwa0_received_desiredEEInRob, delimiter=' ')
			#print "done"
			
			
            

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()



def quaternion(R):
	m00 = R[0,0]; 
	m01 = R[0,1]; 
	m02 = R[0,2];
	m10 = R[1,0]; 
	m11 = R[1,1]; 
	m12 = R[1,2];
	m20 = R[2,0]; 
	m21 = R[2,1]; 
	m22 = R[2,2];

    
	tr = m00 + m11 + m22
	# check if w is real. Otherwise, zero it.
	if( tr > 0 ):
		S = math.sqrt(tr+1.0) * 2; 
		qw = 0.25 * S;
		qx = (m21 - m12) / S;
		qy = (m02 - m20) / S; 
		qz = (m10 - m01) / S; 		
	elif ((m00 > m11)&(m00 > m22)):
		S = math.sqrt(1.0 + m00 - m11 - m22) * 2; 
		qw = (m21 - m12) / S;
		qx = 0.25 * S;
		qy = (m01 + m10) / S; 
		qz = (m02 + m20) / S; 
	elif (m11 > m22):
		S = math.sqrt(1.0 + m11 - m00 - m22) * 2;
		qw = (m02 - m20) / S;
		qx = (m01 + m10) / S; 
		qy = 0.25 * S;
		qz = (m12 + m21) / S; 
	else:
		S = math.sqrt(1.0 + m22 - m00 - m11) * 2;
		qw = (m10 - m01) / S;
		qx = (m02 + m20) / S;
		qy = (m12 + m21) / S;
		qz = 0.25 * S;


	return [qw, qx, qy, qz]


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


    


