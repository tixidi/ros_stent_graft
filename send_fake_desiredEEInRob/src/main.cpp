#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "functions.h"
#include "definitions.h"
//#include "Erl.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <iiwa_test/iiwaState.h>


bool iiwa0_reached = false;
bool iiwa1_reached = false;
bool exotica_complete = false;
int iiwa0_state_count = -1;
int iiwa1_state_count = -1;


void posCallback_iiwa0_state(const iiwa_test::iiwaState::ConstPtr& msg){
    if (msg->header.seq-iiwa0_state_count == 1){
        iiwa0_state_count = msg->header.seq;
        iiwa0_reached = (bool)(msg->iiwaReached);
    }
}
void posCallback_iiwa1_state(const iiwa_test::iiwaState::ConstPtr& msg){
    if (msg->header.seq-iiwa1_state_count == 1){
        iiwa1_state_count = msg->header.seq;
        iiwa1_reached = (bool)(msg->iiwaReached);
    }
}
void posCallback_iiwa0_reached(const std_msgs::Bool::ConstPtr& msg){
      iiwa0_reached = msg->data;
}
void posCallback_iiwa1_reached(const std_msgs::Bool::ConstPtr& msg){
      iiwa1_reached = msg->data;

}
void posCallback_exotica_complete(const std_msgs::Bool::ConstPtr& msg){
	exotica_complete = (bool)msg->data;
	cout<<"exotica_complete "<<exotica_complete<<endl;
}
void cam2robFrame(int iiwaNo, double *iiwa_desiredEEInCam, double* desiredEEInRob){

	Matrix4d cHtool;
	for (int i=0; i<3; i++){
		for (int j=0; j<4; j++){
			cHtool(i,j) = iiwa_desiredEEInCam[i*4+j];
		}
	}
	cHtool(3,3) = 1;	
	Matrix4d eeHtool;
	if (iiwaNo == 0){
		eeHtool = Functions::readTransformEigen("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/SutureInEE.txt");	
	}
	if (iiwaNo == 1){
		eeHtool = Functions::readTransformEigen("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/ToolRInEE.txt");	
	}
	
	Matrix4d cHr = Functions::readTransformEigen(strdup(("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/iiwa"+std::to_string(iiwaNo)+"2CameraTransFile.txt").c_str()));	
	
	
	

	Matrix4d rHee = cHr.inverse()*cHtool*eeHtool.inverse();
	
	for (int i = 0; i < 3; i ++){
		for(int j = 0; j < 4; j ++){
			desiredEEInRob[i*4+j] = rHee(i,j);
		}
	}

	
}

int main(int argc,char **argv)
{
	
	ros::init(argc, argv, "send_fake_desiredEEInRob");
    ros::NodeHandle nh;
	ros::Publisher pub_iiwa0_desiredEEInRob = nh.advertise<std_msgs::Float64MultiArray>("iiwa0_desiredEEInRob", 100, true);
	ros::Publisher pub_iiwa1_desiredEEInRob = nh.advertise<std_msgs::Float64MultiArray>("iiwa1_desiredEEInRob", 100, true);
	ros::Publisher pub_iiwa0_desiredEEInRob_sent = nh.advertise<std_msgs::Bool>("iiwa0_desiredEEInRob_sent", 100, true);
    ros::Publisher pub_iiwa1_desiredEEInRob_sent = nh.advertise<std_msgs::Bool>("iiwa1_desiredEEInRob_sent", 100, true);
    
    
    //ros::Subscriber sub_iiwa0_state = nh.subscribe("iiwa0_state", 1000, posCallback_iiwa0_state);
    //ros::Subscriber sub_iiwa1_state = nh.subscribe("iiwa1_state", 1000, posCallback_iiwa1_state);

    ros::Subscriber sub_iiwa0_reached = nh.subscribe("iiwa0_reached", 1000, posCallback_iiwa0_reached);
    ros::Subscriber sub_iiwa1_reached = nh.subscribe("iiwa1_reached", 1000, posCallback_iiwa1_reached);
	ros::Subscriber sub_exotica_complete = nh.subscribe("exotica_complete", 1000, posCallback_exotica_complete);
	
    ros::Rate rate(300);
	
	//read trajectory 
    //ifstream read_traj_mat("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/toolmandrel_2018-04-19-16-03-46_s.txt_smooth_quat_r002_mat");
    //ifstream read_traj_mat("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/toolmandrel_2018-06-11-23-33-00_s_mat");
	ifstream read_traj_mat("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/send_fake_desiredEEInRob/src/toolmandrel_2018-06-29-16-50.txt_smooth_quat");
	
	//ifstream read_traj_mat("/home/charlie/Documents/workspace/ros_ws/iiwa1_transformd (copy).txt_2018-07-02-18-21");

    double iiwa0_next_desiredEEInCam[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
    double iiwa1_next_desiredEEInCam[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
	int count  = 0;
	exotica_complete = true;
	std_msgs::Float64MultiArray tmp0, tmp1;
	while(true){
		ros::spinOnce();
		
		//cout<<count<<" "<<iiwa1_reached<<" "<<exotica_complete<<endl;
		
        //if ((iiwa0_reached) && exotica_complete){
        
        if (exotica_complete){
			iiwa0_reached = false;
            iiwa1_reached = false;
			exotica_complete = false;
			double iiwa0_next_desiredEEInRob[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
            double iiwa1_next_desiredEEInRob[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
			string str;
			for (int i = 0; i <12; i++){
				read_traj_mat >>str;
			}	
			cout<<count<<endl;		
			//cout<<count<<" next_desiredEEInCam"<<endl;
			for (int i = 0; i <12; i++){
                //read_traj_mat >>str;
                read_traj_mat >>iiwa0_next_desiredEEInCam[i];
				//cout<<next_desiredEEInCam[i]<<" ";
			}
			//cout<<endl;
			for (int i = 0; i <12; i++){
                //read_traj_mat >>str;
                //read_traj_mat >>iiwa1_next_desiredEEInCam[i];
                read_traj_mat >>iiwa1_next_desiredEEInRob[i];
			}
			count++;
			cout<<"cam2robFrame"<<endl;
            cam2robFrame(0, iiwa0_next_desiredEEInCam, iiwa0_next_desiredEEInRob);
            cam2robFrame(1, iiwa1_next_desiredEEInCam, iiwa1_next_desiredEEInRob);

            
			cout<<"next_desiredEEInRob"<<endl;
			for (int j = 0; j <12; j ++){
                tmp0.data.push_back(iiwa0_next_desiredEEInRob[j]);
                tmp1.data.push_back(iiwa1_next_desiredEEInRob[j]);
				cout<<iiwa0_next_desiredEEInRob[j]<<" ";
			}
			cout<<endl;
			
			cout<<"tmp0"<<endl<<tmp0<<endl;	
			
            pub_iiwa0_desiredEEInRob.publish(tmp0);
            pub_iiwa0_desiredEEInRob_sent.publish(true);
            rate.sleep();
            pub_iiwa1_desiredEEInRob.publish(tmp1);
            pub_iiwa1_desiredEEInRob_sent.publish(true);
			rate.sleep();
		}

		
	}		
	
	
	
	return 0;
}

