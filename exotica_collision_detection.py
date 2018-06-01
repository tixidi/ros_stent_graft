#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

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
##########################
Tau = 0.05
length = 2

iiwa0_desiredEEInRob = [-8.642523e-01, -4.850827e-01, 1.332769e-01, 8.657560e-02, -1.584119e-04, -2.646708e-01, -9.643388e-01, 8.065320e-03, 5.030586e-01, -8.334531e-01, 2.286655e-01, 1.536100e-01]
iiwa0_desiredEEInRob1 = [-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000];

iiwa1_desiredEEInRob = [9.589071e-01, 1.586790e-01, 2.351980e-01, -1.816500e-02, -2.079816e-01, -1.707150e-01, 9.631199e-01, 1.618970e-02, 1.929788e-01, -9.724594e-01, -1.306975e-01, 1.794060e-01]
iiwa1_desiredEEInRob1 = [8.393542e-01, 3.766504e-01, 3.919426e-01, -1.025624e-02, -2.121405e-01, -4.368858e-01, 8.741437e-01, 1.876013e-02, 5.004807e-01, -8.168631e-01, -2.867992e-01, 1.733888e-01]

man_pose = [-6.441864e-02, 3.629732e-01, -9.295702e-01, 2.398620e-02, -9.953611e-01, 4.333241e-02, 8.589810e-02, -3.385610e-02, 7.145922e-02, 9.307915e-01, 3.584980e-01, 2.146500e-01]
man_pose1 = [-6.441864e-02, 3.629732e-01, -9.295702e-01, 2.398620e-02, -9.953611e-01, 4.333241e-02, 8.589810e-02, -3.385610e-02, 7.145922e-02, 9.307915e-01, 3.584980e-01, 2.146500e-01]

data_man_tool = np.zeros(shape=(2,36))
data_man_tool[0,0:12] = man_pose;
data_man_tool[0,12:24] = iiwa0_desiredEEInRob;
data_man_tool[0,24:36] = iiwa1_desiredEEInRob;

data_man_tool[1,0:12] = man_pose1;
data_man_tool[1,12:24] = iiwa0_desiredEEInRob1;
data_man_tool[1,24:36] = iiwa1_desiredEEInRob1;

iiwa0_reached = True;
desiredEEInRob_received = True;

def callback_desiredEEInRob_received(data):
  return

def callback_iiwa0_desiredEEInRob(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
		#print data.data
		iiwa0_desiredEEInRob = data.data;
		data_man_tool[1,12:24] = np.array(np.asarray(list(data.data)))

def callback_iiwa1_desiredEEInRob(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
		print data.data

def callback_iiwa0_reached(data):
		global iiwa0_reached
		#print data.data
		iiwa0_reached = data.data		
    
def main():
    global iiwa0_reached
    global desiredEEInRob_received
		#Initialise ros
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('exotica_collision_detection', anonymous=True)
    rate = rospy.Rate(10) # 10hz

		#publisher 
    pub_iiwa0_destJoints = rospy.Publisher('iiwa0_destJoints', JointState, queue_size=10)
    pub_exotica_complete = rospy.Publisher('exotica_complete', Bool, queue_size=10)
    
		#pub_iiwa1_destJoints = rospy.Publisher('iiwa1_destJoints', JointState, queue_size=10)

		#subscriber
    rospy.Subscriber("iiwa0_desiredEEInRob", Float64MultiArray, callback_iiwa0_desiredEEInRob)
    rospy.Subscriber("iiwa0_reached", Bool, callback_iiwa0_reached)
    #rospy.Subscriber("desiredEEInRob_received", Bool, callback_desiredEEInRob_received)
		#rospy.Subscriber("iiwa1_desiredEEInRob", Float64MultiArray, callback_iiwa1_desiredEEInRob)
    

    while(1):
        if desiredEEInRob_received and not np.array_equal(data_man_tool[1,12:24],[-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000]):
            desiredEEInRob_received = False
            print data_man_tool[1,12:24]
            exo.Setup.initRos()
            write_smoothed_traj(0,0)
            solver=exo.Setup.loadSolver('{stentgraft_sewing_planning}/resources/aico_trajectory.xml')
            problem = solver.getProblem()

            for t in range(0,problem.T):
                problem.setRho('Frame0',1e5,t)
                problem.setRho('Frame1',1e5,t)

            solution = solver.solve()
            if not is_trajectory_valid(problem, solution):
                print 'Trajectory invalid'
            
            #save_trajectory(solution)
            #print type(solution)
            
            i = 0;	
            
            print np.size(solution,0)
            while(i<np.size(solution,0)):
              
              msg = JointState()
              msg.name = ["iiwa0 joint 1","iiwa0 joint 2","iiwa0 joint 3","iiwa0 joint 4","iiwa0 joint 5","iiwa0 joint 6","iiwa0 joint 7"]
              msg.position = solution[i,0:7]
              #print msg.position
              pub_iiwa0_destJoints.publish(msg)
              rate.sleep()
              
              print i;
              if(iiwa0_reached):
                i = i + 1;
                iiwa0_reached = False
            

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()





    
def write_smoothed_traj(delta_mx, delta_mz):
	# Read trajectory
	pack_path = exo.Tools.parsePath('{stentgraft_sewing_planning}/resources/')
	# print 'Reading ', length, ' poses'
	man = data_man_tool[:,0:12]
	toolL = data_man_tool[:,12:24]
	toolR = data_man_tool[:,24:36]

	# Read cHi0 cHi1 i0Hm i1Hm
	cHi0 = np.loadtxt(pack_path + 'iiwa02CameraTransFile.txt')
	cHi1 = np.loadtxt(pack_path + 'iiwa12CameraTransFile.txt')
	eeHl = np.loadtxt(pack_path + 'SutureInEE.txt')
	eeHr = np.loadtxt(pack_path + 'ToolRInEE.txt')

	mHm_ = np.eye(4,4)
	mHm_[0,3] = delta_mx
	cHdm = np.eye(4,4)
	cHdm[1,3] = -delta_mx
	cHdm[2,3] = delta_mz
	cHm = np.eye(4,4)
	cHl = np.eye(4,4)
	cHr = np.eye(4,4)

	iiwa0_traj = np.zeros((length, 8))
	iiwa1_traj = np.zeros((length, 8))

	for i in range(0, length):
		cHm[0,0:4] = man[i,0:4]
		cHm[1,0:4] = man[i,4:8]
		cHm[2,0:4] = man[i,8:12]

		cHl[0,0:4] = toolL[i,0:4]
		cHl[1,0:4] = toolL[i,4:8]
		cHl[2,0:4] = toolL[i,8:12]

		cHr[0,0:4] = toolR[i,0:4]
		cHr[1,0:4] = toolR[i,4:8]
		cHr[2,0:4] = toolR[i,8:12]

		# cHm_ = np.matmul(cHm, mHm_)
		cHm_ = np.matmul(cHdm, cHm)
		mHl  = np.matmul(inv(cHm), cHl)
		cHl_ = np.matmul(cHm_, mHl)
		mHr  = np.matmul(inv(cHm), cHr)
		cHr_ = np.matmul(cHm_, mHr)

		#i0Hee = np.matmul(np.matmul(inv(cHi0),cH.3l_), inv(eeHl))
		#i1Hee = np.matmul(np.matmul(inv(cHi1),cHr_), inv(eeHr))
		cHee0 = np.matmul(cHl_, inv(eeHl))
		cHee1 = np.matmul(cHr_, inv(eeHr))

		#Re-orthogonal matrix
		#u, s, vh = np.linalg.svd(i0Hee[0:3,0:3], full_matrices=True)
		#i0Hee[0:3,0:3] = np.matmul(u, vh)
		#u, s, vh = np.linalg.svd(i1Hee[0:3,0:3], full_matrices=True)
		#i1Hee[0:3,0:3] = np.matmul(u, vh)
		u, s, vh = np.linalg.svd(cHee0[0:3,0:3], full_matrices=True)
		cHee0[0:3,0:3] = np.matmul(u, vh)
		u, s, vh = np.linalg.svd(cHee1[0:3,0:3], full_matrices=True)
		cHee1[0:3,0:3] = np.matmul(u, vh)

		#toolL_quat = Quaternion(matrix=i0Hee)
		#toolR_quat = Quaternion(matrix=i1Hee)

		toolL_quat = Quaternion(matrix=cHee0)
		toolR_quat = Quaternion(matrix=cHee1)

		iiwa0_traj[i, 1] = cHee0[0,3]
		iiwa0_traj[i, 2] = cHee0[1,3]
		iiwa0_traj[i, 3] = cHee0[2,3]
		iiwa0_traj[i, 4] = toolL_quat[1]
		iiwa0_traj[i, 5] = toolL_quat[2]
		iiwa0_traj[i, 6] = toolL_quat[3]
		iiwa0_traj[i, 7] = toolL_quat[0]

		iiwa1_traj[i, 1] = cHee1[0, 3]
		iiwa1_traj[i, 2] = cHee1[1, 3]
		iiwa1_traj[i, 3] = cHee1[2, 3]
		iiwa1_traj[i, 4] = toolR_quat[1]
		iiwa1_traj[i, 5] = toolR_quat[2]
		iiwa1_traj[i, 6] = toolR_quat[3]
		iiwa1_traj[i, 7] = toolR_quat[0]

		#dt = 10.0 / (length - 1.0)
		dt = 10;
		for i in range(0, length):
			iiwa0_traj[i, 0] = i * dt
			iiwa1_traj[i, 0] = i * dt

	np.set_printoptions(threshold='nan')

	tmp = '1\n' + str(length) + '\t8\n'
	iiwa0_traj = np.array2string(iiwa0_traj, separator='\t', max_line_width=np.inf)
	iiwa1_traj = np.array2string(iiwa1_traj, separator='\t', max_line_width=np.inf)

	iiwa0_traj = iiwa0_traj.replace('[', '')
	iiwa0_traj = iiwa0_traj.replace(']', '')
	iiwa0_traj = iiwa0_traj.replace(' ', '')

	iiwa1_traj = iiwa1_traj.replace('[', '')
	iiwa1_traj = iiwa1_traj.replace(']', '')
	iiwa1_traj = iiwa1_traj.replace(' ', '')

	text_file = open(pack_path + 'iiwa_0.traj', "w")
	text_file.write(tmp + iiwa0_traj)
	text_file.close()

	text_file = open(pack_path + 'iiwa_1.traj', "w")
	text_file.write(tmp + iiwa1_traj)
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


if __name__ == '__main__':
    main()


    


