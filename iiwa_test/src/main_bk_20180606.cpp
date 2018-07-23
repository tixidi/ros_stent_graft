#include <ros/ros.h>
#include <iiwa_test/ToolsPose.h> 
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"

#include <kukasunrise.h>
#include <kukasunriseplanned.h>
#include <fstream>
#include <iostream>

//#include <ctime>
//#include <chrono>
#include <time.h>

using namespace std;
using namespace std::chrono;

Eigen::Matrix<double, 7, 1> destJoints;
bool newDest = false;
bool iiwa_currMsrTransform_received = false;
bool iiwa_connected = false;
/*void posCallback(const sensor_msgs::JointState::ConstPtr& msg){
		cout<<endl;
		cout<<"destJoints ";
		for (int i = 0; i < 7; i ++){
				destJoints[i] = msg->position[i];
				cout<<destJoints[i]<<" ";
		}
		cout<<endl;
		cout<<"------------------------------------------------------------------------------"<<endl;
		newDest = true;

}*/

void posCallback_destJoints(const sensor_msgs::JointState::ConstPtr& msg){
		cout<<endl;
		cout<<"destJoints ";
		for (int i = 0; i < 7; i ++){
				destJoints[i] = msg->position[i];
				cout<<destJoints[i]<<" ";
		}
		cout<<endl;
		cout<<"------------------------------------------------------------------------------"<<endl;
		newDest = true;

}

void posCallback_iiwa_currMsrTransform_received(const std_msgs::Bool::ConstPtr& msg){
	iiwa_currMsrTransform_received = msg->data;
}

int main(int argc,char **argv)
{
////////////////
	if (argc<3 || argc>3) {
		cerr<<"Invalid number of arguments"<<endl;
		cerr<<"Usage: [kuka number] [mode 0:kuka 1:test]"<<endl;
		return 1;
    }
	if (int(atoi(argv[1]))>1 ||int(atoi(argv[1]))<0 ){
			cerr<<"Invalid kuka number"<<endl;
			return 1;
	}
	if (int(atoi(argv[2]))>1 ||int(atoi(argv[2]))<0 ){
			cerr<<"Invalid mode"<<endl;
			return 1;
	}
	//initialise ros 
	string kukaNo = argv[1];
	string nodeName = "kuka"+kukaNo;
	ros::init(argc, argv, nodeName);
    ros::NodeHandle nh;
	//ros::Publisher pub=nh.advertise<iiwa_test::ToolsPose>("kuka_pose", 100);
	
	//publisher 
	ros::Publisher pub1=nh.advertise<iiwa_test::ToolsPose>("kuka"+kukaNo+"_pose", 100);
	ros::Publisher pub2=nh.advertise<std_msgs::Bool>("kuka"+kukaNo+"_pose_done", 100);
	ros::Publisher pub_iiwa_reached=nh.advertise<std_msgs::Bool>("iiwa"+kukaNo+"_reached", 100);
	ros::Publisher pub_iiwa_connected=nh.advertise<std_msgs::Bool>("iiwa"+kukaNo+"_connected", 100);
	ros::Publisher pub_iiwa_msrTransform=nh.advertise<std_msgs::Float64MultiArray>("iiwa"+kukaNo+"_msrTransform", 100);
	ros::Publisher pub_iiwa_currMsrTransform_sent=nh.advertise<std_msgs::Bool>("iiwa"+kukaNo+"_currMsrTransform_sent", 100);
	ros::Publisher pub_iiwa_currMsrTransform_received=nh.advertise<std_msgs::Bool>("iiwa"+kukaNo+"_currMsrTransform_received", 100);

	//subscriber
	//string topicName = "kuka"+kukaNo+"_command";
	//ros::Subscriber sub = nh.subscribe(topicName, 1000, posCallback);
	string topicName_destJoints = "iiwa"+kukaNo+"_destJoints";
	ros::Subscriber sub = nh.subscribe(topicName_destJoints, 1000, posCallback_destJoints);
	ros::Subscriber sub_iiwa_currMsrTransform_received = nh.subscribe("iiwa"+kukaNo+"_currMsrTransform_received", 1000, posCallback_iiwa_currMsrTransform_received);

	srand(time(0));
    ros::Rate rate(10);

	pub_iiwa_connected.publish(false);
	int mode = int(atoi(argv[2]));// 0:kuka mode 1:test mode
	if (mode == 0)
		cout<<"kuka mode on"<<endl;
	if (mode == 1)
		cout<<"test mode on"<<endl;		

	//kuka initialisation
	KukaSunrisePlanned kuka;
	KukaSunrisePlanned::InitialParameters params;
	if (mode == 0){
		if (std::stoi(kukaNo)==0){
			params.hostname_ = "192.170.10.105";
			//params.hostname_ = "192.170.10.99";
			params.Id_ = "SuturingKuka"+kukaNo;
			params.localPort_ = 9875;
		}else if(std::stoi(kukaNo)==1){
			params.hostname_ = "192.170.10.106";
			params.Id_ = "SuturingKuka"+kukaNo;
			params.localPort_ = 9876;

		}else{
			return 0;
		}
		cout<<"print host: "<<params.hostname_<<endl;

		params.iiwaPort_ = 18000;
		params.comm_timeout_ = 1000;
		params.inital_timeout_ = 6000;
		params.plannerSleepTime_ = 3;
		params.plannerCycleTime_ = 5;

		params.maxVelocity_ = Erl::Vector6d(200, 200, 200, 20, 20, 20);
		params.maxAcceleration_ = Erl::Vector6d(200, 200, 200, 20, 20, 20);
		params.maxJointVelocity_ = Eigen::Matrix<double, 7, 1>();
		params.maxJointVelocity_ << 20,20,20,20,20,20,20;
		params.maxJointAcceleration_ = params.maxJointVelocity_;

		bool iiwaConnected = kuka.start(params);
		Erl::sleep_ms(2000);


		if (iiwaConnected == false)
		{
			cout << "iiwa is not connected!" << endl;
			return (0);
		}
		else
		{
			iiwa_connected = true;
			cout << "iiwa " << params.hostname_ << " is connected!" << endl;
		}

	}

////////////////////////

		double freq = 10; //hz
		double angSpd = 10*3.14/(180*freq); //in rad/time
		//double angSpd = 0.0227598; //in rad/ms
		Eigen::Matrix<double, 7, 1> currJoints, currTmpJoints;
		if (mode ==0)		
				currJoints = kuka.getJoints();  
		if (mode ==1 && std::stoi(kukaNo)==0)		
				currJoints<<0.4,-0.8,-0.1,1.6,-2.7,-1.6,1.9;
		if (mode ==1 && std::stoi(kukaNo)==1)		
				currJoints<<-1.24,0.7513,0.1361,-1.4, -1.72, -1.35, 0.1125;
		
		std_msgs::Float64MultiArray msg_msrTransform;
		if (mode == 0){
			cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl;
			cout<<kuka.getMsrTransform()<<endl;
			
			for(int i = 0; i < 3; i ++){
				msg_msrTransform.data.push_back(kuka.getMsrTransform().getColumn0()[i]);
				msg_msrTransform.data.push_back(kuka.getMsrTransform().getColumn1()[i]);
				msg_msrTransform.data.push_back(kuka.getMsrTransform().getColumn2()[i]);
				if (i == 0)
						msg_msrTransform.data.push_back(kuka.getMsrTransform().getX()/1000.0);
				if (i == 1)
						msg_msrTransform.data.push_back(kuka.getMsrTransform().getY()/1000.0);
				if (i == 2)
						msg_msrTransform.data.push_back(kuka.getMsrTransform().getZ()/1000.0);																				
			}
		}
		if (mode == 1 && std::stoi(kukaNo)==0){
			double tmp[12] = {-8.642523e-01, -4.850827e-01, 1.332769e-01, 8.657560e-02, -1.584119e-04, -2.646708e-01, -9.643388e-01, 8.065320e-03, 5.030586e-01, -8.334531e-01, 2.286655e-01, 1.536100e-01};
			for(int i = 0; i < 12; i ++)
				msg_msrTransform.data.push_back(tmp[i]);
		}
		
		while(!iiwa_currMsrTransform_received){
			pub_iiwa_msrTransform.publish(msg_msrTransform);
			ros::spinOnce();
			pub_iiwa_currMsrTransform_sent.publish(true);
		}
		iiwa_currMsrTransform_received = false;
		pub_iiwa_currMsrTransform_received.publish(false);

		ofstream outfile,outfile1;
		outfile.open("int_trajectory.txt");
		outfile1.open("int_trajectory1.txt");
		outfile<<currJoints[0]<<" "<<currJoints[1]<<" "<<currJoints[2]<<" "<<currJoints[3]<<" "<<currJoints[4]<<" "<<currJoints[5]<<" "<<currJoints[6]<<endl;
		outfile1<<currJoints[0]<<" "<<currJoints[1]<<" "<<currJoints[2]<<" "<<currJoints[3]<<" "<<currJoints[4]<<" "<<currJoints[5]<<" "<<currJoints[6]<<endl;
		//ifstream infile("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/traj_solution");	
		
		pub_iiwa_reached.publish(true);
		pub_iiwa_connected.publish(iiwa_connected);

		while(ros::ok()) {
		
				ros::spinOnce();
				if (newDest){
						pub_iiwa_reached.publish(false);
						newDest = false;
		
						double maxDst = 0; //in rad
						for (int i = 0; i < 7; i++){
								if(maxDst<abs(destJoints[i]-currJoints[i])){
										maxDst = abs(destJoints[i]-currJoints[i]);
								}
						}
						double maxTime = maxDst/angSpd;	
						
///////////////////////////						
						//for (int tt = 1; tt < round(maxTime);tt++){
						double tt = 1;
						while (tt < maxTime){
								Eigen::Matrix<double, 7, 1> newJoints;
								
								for (int j = 0; j<7;j++){
										newJoints[j] = currJoints[j] + (destJoints[j]-currJoints[j])*double(tt)/maxTime;
										outfile<<newJoints[j]<<" ";	
										cout<<newJoints[j]<<" ";	
										//outfile1<<"0"<<" ";
								}
								outfile<<endl;
								//outfile1<<endl;
								cout<<endl;	
								tt+=1;
								

								if (mode ==0){
										kuka.setJoints(newJoints);
										//std::this_thread::sleep_for(std::chrono::milliseconds(1));
										currTmpJoints = kuka.getJoints();
										for (int j = 0; j<7;j++){
											outfile1<<currTmpJoints[j]<<" ";
										}
										outfile1<<endl;
								}
								if (mode ==1)
										currTmpJoints = newJoints;
				//ROS: ---------------------------------------------------------------------
								
								iiwa_test::ToolsPose msg;
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 1");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 2");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 3");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 4");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 5");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 6");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 7");
				
								for (int i = 0; i <7; i++){
										if (std::stoi(kukaNo)==0){
												msg.tool.position.push_back(currTmpJoints[i]);
										}
										if (std::stoi(kukaNo)==1){
												msg.tool.position.push_back(currTmpJoints[i+7]);
										}
								}

								pub1.publish(msg);
								rate.sleep();
							}	

/*
						//for (int tt = 1; tt < round(maxTime);tt++){
						double tt = 1;
						while (tt < round(maxTime)){
								Eigen::Matrix<double, 7, 1> newJoints;
								for (int j = 0; j<7;j++){
										newJoints[j] = currJoints[j] + (destJoints[j]-currJoints[j])*double(tt)/maxTime;
										outfile<<newJoints[j]<<" ";	
										cout<<newJoints[j]<<" ";	
										//outfile1<<"0"<<" ";
								}
								outfile<<endl;
								outfile1<<endl;
								cout<<endl;	
								tt+=1;

								if (mode ==0){
										kuka.setJoints(newJoints);
										//std::this_thread::sleep_for(std::chrono::milliseconds(1));
										currTmpJoints = kuka.getJoints();
										for (int j = 0; j<7;j++){
											outfile1<<currTmpJoints[j]<<" ";
										}
										outfile1<<endl;
								}
								if (mode ==1)
										currTmpJoints = newJoints;
				//ROS: ---------------------------------------------------------------------
								
								iiwa_test::ToolsPose msg;
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 1");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 2");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 3");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 4");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 5");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 6");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 7");
				
								for (int i = 0; i <7; i++){
										if (std::stoi(kukaNo)==0){
												msg.tool.position.push_back(currTmpJoints[i]);
										}
										if (std::stoi(kukaNo)==1){
												msg.tool.position.push_back(currTmpJoints[i+7]);
										}
								}

								pub1.publish(msg);
								rate.sleep();
							}	
*/
				//ROS: ---------------------------------------------------------------------					
							
							for (int i = 0; i < 7;i++){
									//outfile<<destJoints[i]<<" ";	
									//outfile1<<destJoints[i]<<" ";	
							}
							//outfile<<endl;
							//outfile1<<endl;
							
							if (mode==0)						
									currJoints = kuka.getJoints();
							if (mode==1)
									currJoints = currTmpJoints;
							
							cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl;
							cout<<kuka.getMsrTransform()<<endl;
							std_msgs::Float64MultiArray msg_msrTransform;
							for(int i = 0; i < 3; i ++){
									msg_msrTransform.data.push_back(kuka.getMsrTransform().getColumn0()[i]);
									msg_msrTransform.data.push_back(kuka.getMsrTransform().getColumn1()[i]);
									msg_msrTransform.data.push_back(kuka.getMsrTransform().getColumn2()[i]);
									if (i == 0)
											msg_msrTransform.data.push_back(kuka.getMsrTransform().getX()/1000.0);
									if (i == 1)
											msg_msrTransform.data.push_back(kuka.getMsrTransform().getY()/1000.0);
									if (i == 2)
											msg_msrTransform.data.push_back(kuka.getMsrTransform().getZ()/1000.0);																				
							}
							pub_iiwa_msrTransform.publish(msg_msrTransform);
							pub_iiwa_currMsrTransform_sent.publish(true);
							
/*							for(int i = 0; i < 12; i ++){
									cout<<msg_msrTransform.data[i]<<endl;
							}
*/
/*
							kuka.setJoints(destJoints);
							std::this_thread::sleep_for(std::chrono::milliseconds(1));
							currJoints = kuka.getJoints();
							for (int i = 0; i < 7;i++){
									cout<<currJoints[i]<<" ";	
							}
							cout<<endl;

				//ROS: ---------------------------------------------------------------------
								
								iiwa_test::ToolsPose msg;

								msg.tool.name.push_back("kuka"+kukaNo+" Joint 1");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 2");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 3");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 4");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 5");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 6");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 7");
				
								for (int i = 0; i <7; i++){
										if (std::stoi(kukaNo)==0){
												msg.tool.position.push_back(currJoints[i]);
										}
										if (std::stoi(kukaNo)==1){
												msg.tool.position.push_back(currJoints[i+7]);
										}
								}
								pub1.publish(msg);
								rate.sleep();
					
				//ROS: ---------------------------------------------------------------------
				//check if all joints are reached
				int noJointsReached = 0;
				for (int i = 0; i <7; i ++){

						if (abs(destJoints[i]-currJoints[i])>0.005){
								//std_msgs::Bool msg = false;
								pub2.publish(false);
								newDest = true;
								rate.sleep();
								//return 0;
						}
						if (abs(destJoints[i]-currJoints[i])<=0.005){
								noJointsReached++;
						}
				}
				//std_msgs::Bool msg = true;
				if (noJointsReached == 7){
						pub2.publish(true);
				}
				
*/				


			}
pub2.publish(true);
pub_iiwa_reached.publish(true);
//rate.sleep();
				
		}
		outfile.close();
		outfile1.close();
		//infile.close();	

///////////////////////
//linear interpolate
/*
		double freq = 10; //hz
		double angSpd = 10*3.14/(180*freq); //in rad/time
		//double angSpd = 0.0227598; //in rad/ms
		Eigen::Matrix<double, 7, 1> currJoints, currTmpJoints;
		currJoints = kuka.getJoints();  
		//currJoints<<0.4,-0.8,-0.1,1.6,-2.7,-1.6,1.9;


		ofstream outfile,outfile1;
		outfile.open("int_trajectory.txt");
		outfile1.open("int_trajectory1.txt");
		outfile<<currJoints[0]<<" "<<currJoints[1]<<" "<<currJoints[2]<<" "<<currJoints[3]<<" "<<currJoints[4]<<" "<<currJoints[5]<<" "<<currJoints[6]<<endl;
		outfile1<<currJoints[0]<<" "<<currJoints[1]<<" "<<currJoints[2]<<" "<<currJoints[3]<<" "<<currJoints[4]<<" "<<currJoints[5]<<" "<<currJoints[6]<<endl;
		ifstream infile("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/traj_solution");
		//while(ros::ok()) {
				//ros::spinOnce();
				//if (newDest){
						//newDest = false;
		double tmp;
		std::list<Eigen::Matrix<double, 7, 1>> jointsData;
		while(infile){
				//cout<<"current pose"<<endl;
				if (std::stoi(kukaNo)==0){
						infile>>destJoints[0];
						infile>>destJoints[1];
						infile>>destJoints[2];
						infile>>destJoints[3];
						infile>>destJoints[4];
						infile>>destJoints[5];
						infile>>destJoints[6];
						infile>>tmp;
						infile>>tmp;
						infile>>tmp;
						infile>>tmp;
						infile>>tmp;
						infile>>tmp;
						infile>>tmp;
				}else if (std::stoi(kukaNo)==1){
						infile>>tmp;
						infile>>tmp;
						infile>>tmp;
						infile>>tmp;
						infile>>tmp;
						infile>>tmp;
						infile>>tmp;
						infile>>destJoints[0];
						infile>>destJoints[1];
						infile>>destJoints[2];
						infile>>destJoints[3];
						infile>>destJoints[4];
						infile>>destJoints[5];
						infile>>destJoints[6];
				}else{
						return 0;
				}

				jointsData.push_back(destJoints);
			}
			jointsData.pop_back();
			std::vector<Eigen::Matrix<double, 7, 1>> joints{ std::begin(jointsData), std::end(jointsData) };
			joints.insert (joints.begin(), currJoints);
			joints.insert (joints.end(), destJoints); 

			for (int k = 1; k<joints.size();k++){
					destJoints=joints[k];	

		
						double maxDst = 0; //in rad
						for (int i = 0; i < 7; i++){
								if(maxDst<abs(destJoints[i]-currJoints[i])){
										maxDst = abs(destJoints[i]-currJoints[i]);
								}
						}
						double maxTime = maxDst/angSpd;	

						for (int tt = 1; tt < round(maxTime);tt++){
								Eigen::Matrix<double, 7, 1> newJoints;
								for (int j = 0; j<7;j++){
										newJoints[j] = currJoints[j] + (destJoints[j]-currJoints[j])*double(tt)/maxTime;
										outfile<<newJoints[j]<<" ";	
										outfile1<<"0"<<" ";
								}
								
								outfile<<endl;
								outfile1<<endl;	
								kuka.setJoints(newJoints);
								std::this_thread::sleep_for(std::chrono::milliseconds(1));
								currTmpJoints = kuka.getJoints();
								//currTmpJoints = newJoints;
				//ROS: ---------------------------------------------------------------------
								
								iiwa_test::ToolsPose msg;
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 1");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 2");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 3");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 4");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 5");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 6");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 7");
				
								for (int i = 0; i <7; i++){
										if (std::stoi(kukaNo)==0){
												msg.tool.position.push_back(currTmpJoints[i]);
										}
										if (std::stoi(kukaNo)==1){
												msg.tool.position.push_back(currTmpJoints[i+7]);
										}
								}

								pub1.publish(msg);
								rate.sleep();
					
				//ROS: ---------------------------------------------------------------------
						
							}
							for (int i = 0; i < 7;i++){
									outfile<<destJoints[i]<<" ";	
									outfile1<<destJoints[i]<<" ";	
							}
							outfile<<endl;
							outfile1<<endl;
							
							//currJoints = destJoints;
							kuka.setJoints(destJoints);
							currJoints = kuka.getJoints();
				//ROS: ---------------------------------------------------------------------
								
								iiwa_test::ToolsPose msg;

								msg.tool.name.push_back("kuka"+kukaNo+" Joint 1");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 2");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 3");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 4");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 5");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 6");
								msg.tool.name.push_back("kuka"+kukaNo+" Joint 7");
				
								for (int i = 0; i <7; i++){
										if (std::stoi(kukaNo)==0){
												msg.tool.position.push_back(currJoints[i]);
										}
										if (std::stoi(kukaNo)==1){
												msg.tool.position.push_back(currJoints[i+7]);
										}
								}
								pub1.publish(msg);
								rate.sleep();

				//ROS: ---------------------------------------------------------------------
				}
		
		outfile.close();
		outfile1.close();
		infile.close();	

*/
///////////////////
//use catmul-rom spline
/*
    double angSpd = 10*3.14/(180*1000); //in rad/ms
    //double angSpd = 0.0227598; //in rad/ms
    Eigen::Matrix<double, 7, 1> currJoints;
    currJoints = kuka.getJoints();
    //currJoints<<-1.238499999999999934e+00, 7.512999999999999678e-01, 1.360999999999999988e-01, -1.396600000000000064e+00, -1.715500000000000025e+00, -1.346200000000000063e+00, 1.125000000000000028e-01;
    //currJoints<<0.4,-0.8,-0.1,1.6,-2.7,-1.6,1.9;
    
    
		Eigen::Matrix<double, 7, 1> destJoints;
    //destJoints << M_PI/2, 0, 0, M_PI/2, 0, -M_PI/2, 0;
		//destJoints<<0.4836,-0.8033,-0.1767,1.6785,-2.7219,-1.6706,1.9363; 
		
		ofstream outfile;
		outfile.open("int_trajectory.txt");
		outfile<<currJoints[0]<<" "<<currJoints[1]<<" "<<currJoints[2]<<" "<<currJoints[3]<<" "<<currJoints[4]<<" "<<currJoints[5]<<" "<<currJoints[6]<<endl;
		
		int count = 0;
		ifstream infile("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/traj_solution");
		double tmp;
		std::list<Eigen::Matrix<double, 7, 1>> jointsData;
		while(infile){
				//cout<<"current pose"<<endl;
				if (std::stoi(kukaNo)==0){
						infile>>destJoints[0];
						infile>>destJoints[1];
						infile>>destJoints[2];
						infile>>destJoints[3];
						infile>>destJoints[4];
						infile>>destJoints[5];
						infile>>destJoints[6];
						infile>>tmp;
						infile>>tmp;
						infile>>tmp;
						infile>>tmp;
						infile>>tmp;
						infile>>tmp;
						infile>>tmp;
				}else if (std::stoi(kukaNo)==1){
						infile>>tmp;
						infile>>tmp;
						infile>>tmp;
						infile>>tmp;
						infile>>tmp;
						infile>>tmp;
						infile>>tmp;
						infile>>destJoints[0];
						infile>>destJoints[1];
						infile>>destJoints[2];
						infile>>destJoints[3];
						infile>>destJoints[4];
						infile>>destJoints[5];
						infile>>destJoints[6];
				}else{
						return 0;
				}

				jointsData.push_back(destJoints);
			}
			jointsData.pop_back();
			std::vector<Eigen::Matrix<double, 7, 1>> joints{ std::begin(jointsData), std::end(jointsData) };
			joints.insert (joints.begin(), currJoints);
			joints.insert (joints.end(), destJoints);
			
			for (int j = 1; j<joints.size()-2;j++){
					
					
					//double maxDst = 0; //in rad
					//for (int i = 0; i < 14; i++){
					//		if(maxDst<abs(destJoints[i]-currJoints[i])){
					//				maxDst = abs(destJoints[i]-currJoints[i]);
					//		}
					//}
					//double maxTime = maxDst/angSpd;	
					

					Eigen::Matrix<double, 7, 1> newJoints;
					//for (double tt = 0; tt < maxTime; tt = tt+maxTime/10.0){
					for (double tt = 0; tt < 1; tt +=0.01){
						for (int i = 0; i < 7; i ++){
							newJoints[i] = 1.0/2.0*(2*joints[j][i]+(-joints[j-1][i]+joints[j+1][i])*tt+(2*joints[j-1][i]-5*joints[j][i]+4*joints[j+1][i]-1*joints[j+2][i])*tt*tt+(-joints[j-1][i]+3*joints[j][i]-3*joints[j+1][i]+joints[j+2][i])*tt*tt*tt);
							outfile<<newJoints[i]<<" ";	
							//currJoints[i]=newJoints[i];
						}
						kuka.setJoints(newJoints);
						currJoints = kuka.getJoints();
						std::this_thread::sleep_for(std::chrono::milliseconds(1));
						currJoints=newJoints;
						outfile<<endl;

//ROS: ---------------------------------------------------------------------

						iiwa_test::ToolsPose msg;
						msg.tool.name.push_back("kuka"+kukaNo+" Joint 1");
						msg.tool.name.push_back("kuka"+kukaNo+" Joint 2");
						msg.tool.name.push_back("kuka"+kukaNo+" Joint 3");
						msg.tool.name.push_back("kuka"+kukaNo+" Joint 4");
						msg.tool.name.push_back("kuka"+kukaNo+" Joint 5");
						msg.tool.name.push_back("kuka"+kukaNo+" Joint 6");
						msg.tool.name.push_back("kuka"+kukaNo+" Joint 7");
						
						for (int i = 0; i <7; i++){
								if (std::stoi(kukaNo)==0){
										msg.tool.position.push_back(currJoints[i]);
								}
								if (std::stoi(kukaNo)==1){
										msg.tool.position.push_back(currJoints[i+7]);
								}
						}

						pub.publish(msg);
						rate.sleep();

//ROS: ---------------------------------------------------------------------
					}
					kuka.setJoints(destJoints);
					currJoints = kuka.getJoints();
			}

		outfile.close();
		infile.close();	
*/		
///////////////////

    cout << "Finished!" << endl;
		if (mode ==0)
    		kuka.stop();
    return 0;
}

