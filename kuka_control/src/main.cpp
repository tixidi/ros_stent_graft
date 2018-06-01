#include <ros/ros.h>
#include "std_msgs/String.h"
#include <track_multitools_markers/ToolsPose.h> 
#include <iiwa_test/ToolsPose.h> 
#include <std_msgs/Int8.h> 
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h> 
#include <sensor_msgs/JointState.h> 
#include <iostream>
#include <string> 
#include <fstream>

#include <stdlib.h>
#include <iostream>
#include <sys/select.h>

using namespace std;
bool kuka0_reached = true;
bool kuka1_reached = true;

track_multitools_markers::ToolsPose toolsPose;
iiwa_test::ToolsPose kuka0Pose, kuka1Pose;

//get mandrel and tools poses
void posCallback1(const track_multitools_markers::ToolsPose::ConstPtr& msg){

		//cout<<msg->toolLTwist<<endl;
		//cout<<msg->toolRTwist<<endl;
		//cout<<msg->mandrelTwist<<endl;
		toolsPose.toolLTwist = msg->toolLTwist;
		toolsPose.toolRTwist = msg->toolRTwist;
		toolsPose.mandrelTwist = msg->mandrelTwist;
}

//get suture device angle
void posCallback2(const std_msgs::Float32::ConstPtr& msg){
	//cout<<"suture device angle "<<msg->data<<endl;
}

//get Kuka0 current joints
void posCallback3(const iiwa_test::ToolsPose::ConstPtr& msg){
	//cout<<msg->tool<<endl;
	kuka0Pose.tool = msg->tool;	
}

//get Kuka1 current joiints
void posCallback4(const iiwa_test::ToolsPose::ConstPtr& msg){
	//cout<<msg->tool<<endl;
	kuka1Pose.tool = msg->tool;
}

//get kuka0 current reach status
void posCallback5(const std_msgs::Bool::ConstPtr& msg){
	cout<<msg->data<<endl;
	if(msg->data)
			kuka0_reached = true;
}

//get kuka1 current reach status
void posCallback6(const std_msgs::Bool::ConstPtr& msg){
	cout<<msg->data<<endl;
	if(msg->data)	
			kuka1_reached = true;
}

//wait for key input
int kbhit(void){
		struct timeval tv;
		fd_set read_fd;

		/* Do not wait at all, not even a microsecond */
		tv.tv_sec=0;
		tv.tv_usec=0;

		/* Must be done first to initialize read_fd */
		FD_ZERO(&read_fd);

		/* Makes select() ask if input is ready:
		* 0 is the file descriptor for stdin */
		FD_SET(0,&read_fd);

		/* The first parameter is the number of the
		* largest file descriptor to check + 1. */
		if(select(1, &read_fd,NULL, /*No writes*/NULL, /*No exceptions*/&tv) == -1)
		return 0; /* An error occured */

		/* read_fd now holds a bit map of files that are
		* readable. We test the entry for the standard
		* input (file 0). */

		if(FD_ISSET(0,&read_fd))
		/* Character pending on stdin */
		return 1;

		/* no characters were pending */
		return 0;
}

int main(int argc, char **argv) {
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "kuka_control");
    ros::NodeHandle nh;

		ros::Publisher pub1 = nh.advertise<std_msgs::Int8>("video_record_command", 100);
		ros::Publisher pub2 = nh.advertise<std_msgs::Bool>("suture_device_command", 100);
		//ros::Publisher pub3 = nh.advertise<sensor_msgs::JointState>("kuka0_command", 100);
		//ros::Publisher pub4 = nh.advertise<sensor_msgs::JointState>("kuka1_command", 100);		
		ros::Subscriber sub1 = nh.subscribe("tools_pose", 1000, posCallback1);
		ros::Subscriber sub2 = nh.subscribe("read_suture_device_angle", 1000, posCallback2);
		ros::Subscriber sub3 = nh.subscribe("kuka0_pose", 1000, posCallback3);
		ros::Subscriber sub4 = nh.subscribe("kuka1_pose", 1000, posCallback4);
		ros::Subscriber sub5 = nh.subscribe("kuka0_pose_done", 1000, posCallback5);
		ros::Subscriber sub6 = nh.subscribe("kuka1_pose_done", 1000, posCallback6);
		srand(time(0));
		ros::Rate rate(10);
		
		//checking for number of arugments
		if (argc<2){
        cerr<<"Invalid number of arguments"<<endl;
        cerr<<"Usage: [mode 0:kuka 1:simulation]"<<endl;
        return 1;
		}

		//setting up files to record markers pose and kuka joints 
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
		char buffer [80];
		string strTemp;
		ofstream markersFile, kukaJointsFile;
		strftime (buffer,80,"%Y-%m-%d-%H-%M",now);
		strTemp = "markersPose_" + string(buffer) + ".txt";
		markersFile.open(strTemp .c_str());
		strTemp = "kukaJointsPose_" + string(buffer) + ".txt";
		kukaJointsFile.open(strTemp .c_str());



		//perform optimisation
		//system("python /home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/scripts/example_aico_trajectory_cost_mul.py");

		int mode = int(stoi(argv[1])); //0:kuka 1:simulation
		
		if (mode==0){
				
				//generate trajectory and read trajectory
				//system("python /home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/scripts/example_aico_trajectory.py 1");

				ifstream infile("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/traj_solution");
				double tmp;
				std::vector<vector<double>> jointsPose;
				int count0 = 0,count1 = 0, update_rate = 10, count = 0;;
				string kukaNo = "2";//////////////////////////////////////////////////////////////////////////////change kuka number 0:iiwa0 1:iiwa1 2:both iiwa
				char line[1024] = {0};
				while(infile){
	
						vector<double> tmpJoints;
						if (std::stoi(kukaNo)==0){
								for (int i = 0; i <7; i ++){
										infile>>tmp;
										tmpJoints.push_back(tmp);
								}
								for (int i = 0; i <7; i ++){
										infile>>tmp;
								}

						}else if (std::stoi(kukaNo)==1){
								for (int i = 0; i <7; i ++){
										infile>>tmp;
								}
								for (int i = 0; i <7; i ++){
										infile>>tmp;
										tmpJoints.push_back(tmp);
								}
						}else{
								for (int i = 0; i <14; i ++){
										infile>>tmp;
										tmpJoints.push_back(tmp);
										//cout<<tmp << " ";
								}
						}
				
						jointsPose.push_back(tmpJoints);
						
					}
					jointsPose.pop_back();


		/*			cout<<jointsData.size()<<endl;
						for(int j = 0; j <jointsData.size();j++)
						for (int i = 0; i <7; i ++){
								cout<<jointsData[j][i]<<" ";
						}
						cout<<endl;
						}
				
		/*
		/*			for (int i = 0;i<jointsData.size();i++){
							for(int j = 0; j <7; j++)
									cout<<jointsData.front()[j]<<" ";
							cout<<endl;
					}
		*/			/*
					jointsData.pop_back();
					std::vector<double*> jointsPose{ std::begin(jointsData), std::end(jointsData) }; 
					cout<<jointsPose.size()<<endl;
					cout<<jointsPose[0][0]<<endl;
					for (int i = 0;i<jointsPose.size();i++){
							for(int j =0; j <7; j++){
					
									cout<<jointsPose[i][j]<<" ";
							}
							cout<<endl;
					}
		*/

				while(ros::ok()) {
					ros::spinOnce();
					if (kbhit()){
						int key = cin.get();
						if (key==int('r')||key==int('s')||key==int('1')||key==int('2')){
							std_msgs::Int8 msg;	
							msg.data = key;
							cout<<"key pressed is "<<int(msg.data)<<endl;
							pub1.publish(msg);
							/*if (key==int('r')){
									std_msgs::Bool kuka0StartMsg,kuka1StartMsg;	
									kuka0StartMsg.data = true;
									kuka1StartMsg.data = true;
									pub5.publish(kuka0StartMsg);
									pub6.publish(kuka1StartMsg);
							}*/
						}
						//////////////////////////////////////////
						else if (key==int('d')){
							std_msgs::Bool msg;	
							msg.data = true;
							//cout<<"bool value "<<msg.data<<endl;
							cout<<"run single stitch"<<endl;
							pub2.publish(msg);
						}

					}
/*
						if (count0<jointsPose.size()){
							//if (kuka0_reached && kuka1_reached){
							if (kuka0_reached){
									sensor_msgs::JointState msg;	
									for (int i = 0; i <7; i ++){
											msg.name.push_back("kuka0 joint "+to_string(i));
											msg.position.push_back(jointsPose[count0][i]);
											cout<<jointsPose[count0][i]<<" ";
									}
									cout<<endl;
									rate.sleep();
									pub3.publish(msg);
									kuka0_reached = false;
									count0++;
							}
						}


						if (count0<jointsPose.size()){
							//if (kuka0_reached && kuka1_reached){
							//if (kuka1_reached){
									sensor_msgs::JointState msg;	
									for (int i = 0; i <7; i ++){
											msg.name.push_back("kuka1 joint "+to_string(i));
											msg.position.push_back(jointsPose[count0][i+7]);
											//cout<<count0<<" " << jointsPose[count0][i+7]<<" ";
									}
									cout<<endl;
									rate.sleep();
									pub4.publish(msg);
									kuka1_reached = false;
									count0++;
							}
*/
						//}


					//if (kuka0_reached && kuka1_reached)
							//count0++;

				}
    
		

		}
		if (mode == 1){
				system("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/set_source_path.sh && roslaunch stentgraft_sewing_planning PythonPlanAICOTrajectory.launch");
		}
  return 0;

}
