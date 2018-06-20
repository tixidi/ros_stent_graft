

#include <ros/ros.h>
#include "tf/tf.h"
#include <iiwa_test/iiwaState.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"

#include <kukasunrise.h>
#include <kukasunriseplanned.h>
#include <fstream>
#include <iostream>

#include <ctime>
#include <chrono>
//#include <time.h>

using namespace std;
using namespace std::chrono;

Eigen::Matrix<double, 7, 1> destJoints;
Eigen::Matrix<double, 7, 1> currJoints, currTmpJoints;
bool newDest = false;
bool iiwa_connected = false;
bool iiwa_reached = false;
double iiwa_msrTransform[12];
int iiwa_destJoints_count = -1;
KukaSunrisePlanned kuka;
string kukaNo;

ofstream outfile2;
int counter = 0;
ros::Publisher pub_iiwa_reached;
ros::Publisher pub_iiwa_connected;
ros::Publisher pub_iiwa_msrTransform;
ros::Publisher pub_iiwa_currJoints;
int rate_hz = 300;

/*void posCallback_iiwa_currJoints(const sensor_msgs::JointState::ConstPtr& msg){
	ros::Rate rate(rate_hz);
	sensor_msgs::JointState tmp;
	//cout<<"currJoints"<<endl;
	for (int i = 0; i <7; i++){
		tmp.name.push_back("iiwa"+kukaNo+" Joint "+to_string(i));
		tmp.position.push_back(currJoints[i]);
		//cout<<currJoints[i]<<" ";
	}
	pub_iiwa_currJoints.publish(tmp);
	rate.sleep();
	//cout<<endl;

}*/

void posCallback_destJoints(const sensor_msgs::JointState::ConstPtr& msg){
/*
	if (newDest == true)
		pub_iiwa_reached.publish(false);
	if (newDest == false)
		pub_iiwa_reached.publish(true);

	for (int j = 0; j<7;j++){
		if(destJoints[j] != msg->position[j]){
			cout<<endl;
			cout<<counter<<" destJoints ";
			for (int i = 0; i < 7; i ++){
					destJoints[i] = msg->position[i];
					cout<<destJoints[i]<<" ";
			}
			cout<<endl;
			cout<<"------------------------------------------------------------------------------"<<endl;
			newDest = true;
			counter++;
			return;		
		}	
	}
*/
    if ((msg->header.seq - iiwa_destJoints_count)==1){
        iiwa_destJoints_count = msg->header.seq;
        ros::Rate rate(rate_hz);
        for (int i = 0; i < 7; i ++){
                destJoints[i] = msg->position[i];
                cout<<destJoints[i]<<" ";
        }
        cout<<endl;
        cout<<"------------------------------------------------------------------------------"<<endl;
        newDest = true;
        iiwa_reached = false;
        pub_iiwa_reached.publish(iiwa_reached);
        rate.sleep();
    }
					
/*		
		for (int j = 0; j<7;j++){
			outfile2<<destJoints[j]<<" ";				
		}
		outfile2<<endl;
*/		
		
		/*for (int j = 0; j<7;j++){
			if (destJoints[j]!=prevDestJoints[j]){
				newDest = true;
				for (int i = 0; i<7;i++){
					outfile2<<destJoints[i]<<" ";	
					prevDestJoints[i] = destJoints[i];
				}
				outfile2<<endl;
				break;
			}
		}*/
/*	if (destJoints!=prevDestJoints){
		newDest = true;
		for (int i = 0; i<7;i++){
			outfile2<<destJoints[i]<<" ";	
			prevDestJoints[i] = destJoints[i];
		}
		outfile2<<endl;
	}	
*/
		

}
/*void posCallback_iiwa_currMsrTransform_received(const std_msgs::Bool::ConstPtr& msg){
	iiwa_currMsrTransform_received = msg->data;
}*/


void posCallback_iiwa_reached(const std_msgs::Bool::ConstPtr& msg){
	ros::Rate rate(rate_hz);
	if (msg->data!=iiwa_reached){
		pub_iiwa_reached.publish(iiwa_reached);
		rate.sleep();
		ros::spinOnce();
	}
	pub_iiwa_reached.publish(iiwa_reached);
	rate.sleep();
}

void posCallback_iiwa_connected(const std_msgs::Bool::ConstPtr& msg){
	ros::Rate rate(rate_hz);
	if (msg->data!=iiwa_connected){
		pub_iiwa_connected.publish(iiwa_connected);
		rate.sleep();
		ros::spinOnce();
	}
	pub_iiwa_connected.publish(iiwa_connected);
	rate.sleep();
}

void posCallback_iiwa_msrTransform(const std_msgs::Float64MultiArray::ConstPtr& msg){
/*	
	for (int i = 0; i <12; i ++){
		if (iiwa_msrTransform[0]!=NULL && abs(msg->data[i]-iiwa_msrTransform[i])>0.01){
			std_msgs::Float64MultiArray tmp;
			for (int j = 0; j <12; j ++){
				tmp.data.push_back(iiwa_msrTransform[j]);
				
			}
			pub_iiwa_msrTransform.publish(tmp);
			ros::spinOnce();
			return;
		}
	}
*/	
	ros::Rate rate(rate_hz);
	std_msgs::Float64MultiArray tmp;
	for (int j = 0; j <12; j ++){
		tmp.data.push_back(iiwa_msrTransform[j]);
	
	}
	pub_iiwa_msrTransform.publish(tmp);
	rate.sleep();
	
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
	kukaNo = argv[1];
	string nodeName = "kuka"+kukaNo;
	ros::init(argc, argv, nodeName);
    ros::NodeHandle nh;
	//ros::Publisher pub=nh.advertise<iiwa_test::ToolsPose>("kuka_pose", 100);
	
	//publisher 
	//ros::Publisher pub1=nh.advertise<iiwa_test::ToolsPose>("kuka"+kukaNo+"_pose", 100, true);
	//ros::Publisher pub2=nh.advertise<std_msgs::Bool>("kuka"+kukaNo+"_pose_done", 10, true);
    pub_iiwa_currJoints=nh.advertise<sensor_msgs::JointState>("iiwa"+kukaNo+"_currJoints", 10, true);
	pub_iiwa_reached=nh.advertise<std_msgs::Bool>("iiwa"+kukaNo+"_reached", 100, true);
	pub_iiwa_connected=nh.advertise<std_msgs::Bool>("iiwa"+kukaNo+"_connected", 10, true);
    //pub_iiwa_msrTransform=nh.advertise<std_msgs::Float64MultiArray>("iiwa"+kukaNo+"_msrTransform", 100, true);
    pub_iiwa_msrTransform=nh.advertise<iiwa_test::iiwaState>("iiwa"+kukaNo+"_msrTransform", 100, true);
	//ros::Publisher pub_iiwa_currMsrTransform_sent=nh.advertise<std_msgs::Bool>("iiwa"+kukaNo+"_currMsrTransform_sent", 100, true);
	//ros::Publisher pub_iiwa_currMsrTransform_received=nh.advertise<std_msgs::Bool>("iiwa"+kukaNo+"_currMsrTransform_received", 100, true);

	//subscriber
	//string topicName = "kuka"+kukaNo+"_command";
	//ros::Subscriber sub = nh.subscribe(topicName, 1000, posCallback);
	
    //string topicName_destJoints = "iiwa"+kukaNo+"_destJoints";
    ros::Subscriber sub_iiwa_destJoints = nh.subscribe("iiwa"+kukaNo+"_destJoints", 1000, posCallback_destJoints);
    //ros::Subscriber sub_iiwa_reached = nh.subscribe("iiwa"+kukaNo+"_reached", 1000, posCallback_iiwa_reached);
    //ros::Subscriber sub_iiwa_connected = nh.subscribe("iiwa"+kukaNo+"_connected", 1000, posCallback_iiwa_connected);
    //ros::Subscriber sub_iiwa_msrTransform = nh.subscribe("iiwa"+kukaNo+"_msrTransform", 1000, posCallback_iiwa_msrTransform);
    //ros::Subscriber sub_iiwa_currJoints = nh.subscribe("iiwa"+kukaNo+"_currJoints", 1000, posCallback_iiwa_currJoints);
	//ros::Subscriber sub_iiwa_currMsrTransform_received = nh.subscribe("iiwa"+kukaNo+"_currMsrTransform_received", 1000, posCallback_iiwa_currMsrTransform_received);

	srand(time(0));
    ros::Rate rate(rate_hz);

	int mode = int(atoi(argv[2]));// 0:kuka mode 1:test mode
	if (mode == 0)
		cout<<"kuka mode on"<<endl;
	if (mode == 1)
		cout<<"test mode on"<<endl;		

	//kuka initialisation
	//KukaSunrisePlanned kuka;
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
			cout << "iiwa " << params.hostname_ << " is connected!" << endl;
		}
		iiwa_connected = iiwaConnected;

	}

////////////////////////

		double freq = 10; //hz
		double angSpd = 10*3.14/(180*freq); //in rad/time
		//double angSpd = 0.0227598; //in rad/ms
		
		if (mode ==0)		
				currJoints = kuka.getJoints();  
		if (mode ==1 && std::stoi(kukaNo)==0)		
				currJoints<<0.4,-0.8,-0.1,1.6,-2.7,-1.6,1.9;
		if (mode ==1 && std::stoi(kukaNo)==1)		
				currJoints<<-1.24,0.7513,0.1361,-1.4, -1.72, -1.35, 0.1125;
		
		
		//
		sensor_msgs::JointState msg_currJoints;		
		for (int i = 0; i <7; i++){
			msg_currJoints.name.push_back("iiwa"+kukaNo+" Joint "+to_string(i));
			msg_currJoints.position.push_back(currJoints[i]);
		}
		std_msgs::Float64MultiArray msg_msrTransform;
		if (mode == 0){
			cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl;
			Erl::Transformd kuka_getMsrTransform = kuka.getMsrTransform();
			cout<<kuka_getMsrTransform <<endl;
			for (int i = 0; i < 3; i ++){
				iiwa_msrTransform[4*i] = kuka_getMsrTransform.getColumn0()[i];
				iiwa_msrTransform[4*i+1] = kuka_getMsrTransform.getColumn1()[i];
				iiwa_msrTransform[4*i+2] = kuka_getMsrTransform.getColumn2()[i];
			}
			iiwa_msrTransform[3] = kuka_getMsrTransform.getX()/1000.0;
			iiwa_msrTransform[7] = kuka_getMsrTransform.getY()/1000.0;
			iiwa_msrTransform[11] = kuka_getMsrTransform.getZ()/1000.0;
		
		}
		
		if (mode == 1)
			iiwa_connected = true;
		
		if (mode == 1 && std::stoi(kukaNo)==0){
			//double tmp[12] = {-8.642523e-01, -4.850827e-01, 1.332769e-01, 8.657560e-02, -1.584119e-04, -2.646708e-01, -9.643388e-01, 8.065320e-03, 5.030586e-01, -8.334531e-01, 2.286655e-01, 1.536100e-01};
			double tmp[12] = {-0.18725492,  0.84542901, -0.50018916, -0.59726775, -0.97520191, -0.09886336,  0.19798807, -0.25512601, 0.11793113,  0.52485612,  0.84297739,  0.40726137};
			for (int i = 0; i < 3; i ++){
				iiwa_msrTransform[i] = tmp[i];
			}
			for (int i = 0; i < 3; i ++){
				iiwa_msrTransform[i] = tmp[i];
			}
	
		}
		
		if (mode == 1 && std::stoi(kukaNo)==1){
			double tmp[12] = { 0.15776873, -0.14297075,  0.97707544,  0.37679855, -0.98729879, -0.04151556,  0.15334503, -0.49452684, 0.01863774, -0.98885802, -0.14770564,  0.43736135};	
		}
		
		
		
		iiwa_reached = true;
		for (int i = 0; i < 12; i ++){
			msg_msrTransform.data.push_back(iiwa_msrTransform[i]);
		}
		
        iiwa_test::iiwaState msg_iiwaState;
        for (int i = 0; i <12; i ++){
            msg_iiwaState.transform.data.push_back(iiwa_msrTransform[i]);
        }
        for (int i = 0; i <7; i ++){
            msg_iiwaState.jointState.data.push_back(currJoints[i]);
        }
        pub_iiwa_msrTransform.publish(msg_iiwaState);


	
		pub_iiwa_connected.publish(iiwa_connected);
		pub_iiwa_reached.publish(iiwa_reached);
        //pub_iiwa_msrTransform.publish(msg_msrTransform);
		pub_iiwa_currJoints.publish(msg_currJoints);
        rate.sleep();
        //ros::spinOnce();
		

		cout<<"finished initialisation"<<endl;
		
		
		ofstream outfile1, outfile_received_destJoints, outfile_received_destJoints_int, outfile_kuka_traj, outfile_kuka_msrTransform;
		outfile_kuka_traj.open("kuka_trajectory1.txt");
		outfile2.open("int_trajectory2.txt");
		outfile_received_destJoints.open("received_destJoints.txt");
		outfile_received_destJoints_int.open("received_destJoints_int.txt");
		outfile_kuka_msrTransform.open("kuka_msrTransform.txt");
		
		outfile_kuka_traj<<currJoints[0]<<" "<<currJoints[1]<<" "<<currJoints[2]<<" "<<currJoints[3]<<" "<<currJoints[4]<<" "<<currJoints[5]<<" "<<currJoints[6]<<endl;
		//ifstream infile("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/traj_solution");	
		
		clock_t begin, end;
		while(ros::ok()) {
            pub_iiwa_reached.publish(iiwa_reached);
            rate.sleep();
                ros::spinOnce();
				if (newDest){
					iiwa_reached = false;
					pub_iiwa_reached.publish(iiwa_reached);
					rate.sleep();
					
					
					begin = clock();
					
	
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
								outfile_received_destJoints_int<<destJoints[j]<<" ";
								cout<<newJoints[j]<<" ";	
						}
						outfile_received_destJoints_int<<endl;
						cout<<endl;	
						tt+=1;
						

						if (mode ==0){
								kuka.setJoints(newJoints);
								std::this_thread::sleep_for(std::chrono::milliseconds(10));
								currTmpJoints = kuka.getJoints();
								//std::this_thread::sleep_for(std::chrono::milliseconds(250));
								for (int j = 0; j<7;j++){
									outfile_kuka_traj<<currTmpJoints[j]<<" ";
								}
								outfile_kuka_traj<<endl;
						}
						if (mode ==1)
								currTmpJoints = newJoints;
					}	
						
					//check if all joints are reached
					int noJointsReached = 0;
					if (mode == 0){
						for (noJointsReached = 0; noJointsReached <7; noJointsReached ++){		
							if (abs(destJoints[noJointsReached]-currJoints[noJointsReached])>0.005){
								noJointsReached = 0;
								kuka.setJoints(destJoints);
								std::this_thread::sleep_for(std::chrono::milliseconds(10));
								currJoints = kuka.getJoints();
							}
						}
					}					
					/////////////////////
//							for(int i = 0; i < 12; i ++)
//									cout<<msg_msrTransform.data[i]<<endl;

					if (mode ==0){
						//kuka.setJoints(destJoints);
						currJoints = kuka.getJoints();
						for (int j = 0; j<7;j++){
							outfile_kuka_traj<<currJoints[j]<<" ";
						}
						outfile_kuka_traj<<endl;
					}
					if (mode ==1)
						currJoints = destJoints;
					
					for (int i = 0 ; i <7; i ++){
						outfile_received_destJoints<<destJoints[i]<<" ";
						outfile_received_destJoints_int<<destJoints[i]<<" ";
					}
					outfile_received_destJoints<<endl;
					outfile_received_destJoints_int<<endl;	
					
					
					cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl;
                    Erl::Transformd kuka_getMsrTransform = kuka.getMsrTransform();
                    /*if (mode == 1){
                        tf::TransformListener tf;
                        tf::StampedTransform tfLink2WrtBaseLink;
                        tf.lookupTransform("world_frame", "exotica/Target0", ros::Time(0), tfLink2WrtBaseLink);
                    }*/


					

					//std::this_thread::sleep_for(std::chrono::milliseconds(50));
					cout<<kuka_getMsrTransform <<endl;
					for (int i = 0; i < 3; i ++){
						iiwa_msrTransform[4*i] = kuka_getMsrTransform.getColumn0()[i];
						iiwa_msrTransform[4*i+1] = kuka_getMsrTransform.getColumn1()[i];
						iiwa_msrTransform[4*i+2] = kuka_getMsrTransform.getColumn2()[i];
					}
					iiwa_msrTransform[3] = kuka_getMsrTransform.getX()/1000.0;
					iiwa_msrTransform[7] = kuka_getMsrTransform.getY()/1000.0;
					iiwa_msrTransform[11] = kuka_getMsrTransform.getZ()/1000.0;
					
					int64_t nowms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
					outfile_kuka_msrTransform<<nowms<<" ";
					for (int i = 0; i <12; i ++){
						outfile_kuka_msrTransform<<iiwa_msrTransform[i]<<" ";
					}
					outfile_kuka_msrTransform<<endl;
					
					end = clock();
					newDest = false;
					iiwa_reached = true;
			
					
					iiwa_test::iiwaState msg_iiwaState;
	                for (int i = 0; i <12; i ++){
	                    msg_iiwaState.transform.data.push_back(iiwa_msrTransform[i]);
	                }
	                for (int i = 0; i <7; i ++){
	                    msg_iiwaState.jointState.data.push_back(currJoints[i]);
	                }
	                pub_iiwa_msrTransform.publish(msg_iiwaState);
	                rate.sleep();
					


                    //publish joint state
                    sensor_msgs::JointState tmp_js;
                    for (int i = 0; i <7; i++){
                        tmp_js.name.push_back("iiwa"+kukaNo+" Joint "+to_string(i));
                        tmp_js.position.push_back(currJoints[i]);
                    }
                    pub_iiwa_currJoints.publish(tmp_js);
                    rate.sleep();

                    //publish msrTransform
//                        std_msgs::Float64MultiArray tmp_trans;
//                        for (int j = 0; j <12; j ++){
//                            tmp_trans.data.push_back(iiwa_msrTransform[j]);
//                        }
//                        pub_iiwa_msrTransform.publish(tmp_trans);
//                        rate.sleep();

                    //publish iiwa_reached
                    pub_iiwa_reached.publish(iiwa_reached);
                    rate.sleep();
                    //cout<<iiwa_reached<<" "<<iiwa_connected<<endl;

                    //ros::spinOnce();
                    //rate.sleep();
						



				
				

			}



			//cout<<"time elapse = "<<double(end - begin) / CLOCKS_PER_SEC<<endl;
			//pub2.publish(true);
			//rate.sleep();
	
		}
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

