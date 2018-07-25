#include <ros/ros.h>
#include "std_msgs/String.h"
#include <track_multitools_markers/ToolsPose.h> 
//#include <iiwa_test/ToolsPose.h> 
#include <std_msgs/Int8.h> 
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h> 
#include <sensor_msgs/JointState.h> 
#include <std_msgs//Float64MultiArray.h>
#include <geometry_msgs/Wrench.h>
#include <iostream>
#include <string> 
#include <fstream>

#include <stdlib.h>
#include <iostream>
#include <sys/select.h>
#include <unistd.h>
#include <termios.h>

using namespace std;
bool kuka0_reached = true;
bool kuka1_reached = true;
bool iiwas_vs_reached = false;
bool suture_complete = false;
track_multitools_markers::ToolsPose toolsPose;
//iiwa_test::ToolsPose kuka0Pose, kuka1Pose;

//get mandrel and tools poses
/*void posCallback1(const track_multitools_markers::ToolsPose::ConstPtr& msg){

		//cout<<msg->toolLTwist<<endl;
		//cout<<msg->toolRTwist<<endl;
		//cout<<msg->mandrelTwist<<endl;
		toolsPose.toolLTwist = msg->toolLTwist;
		toolsPose.toolRTwist = msg->toolRTwist;
		toolsPose.mandrelTwist = msg->mandrelTwist;
}*/

//get suture device angle
void posCallback2(const std_msgs::Float32::ConstPtr& msg){
	//cout<<"suture device angle "<<msg->data<<endl;
}

/*//get Kuka0 current joints
void posCallback3(const iiwa_test::ToolsPose::ConstPtr& msg){
	//cout<<msg->tool<<endl;
	kuka0Pose.tool = msg->tool;	
}

//get Kuka1 current joiints
void posCallback4(const iiwa_test::ToolsPose::ConstPtr& msg){
	//cout<<msg->tool<<endl;
	kuka1Pose.tool = msg->tool;
}*/

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

//get kuka0 current reach status
void posCallback_iiwa0_reached (const std_msgs::Bool::ConstPtr& msg){
	//cout<<msg->data<<endl;
	if(msg->data)
		kuka0_reached = true;
}

//get kuka1 current reach status
void posCallback_iiwa1_reached (const std_msgs::Bool::ConstPtr& msg){
	//cout<<msg->data<<endl;
	if(msg->data)	
		kuka1_reached = true;
}

//read mandrel force
void posCallback_read_force (const geometry_msgs::Wrench::ConstPtr& msg){

}

//iiwas_vs_reached
void posCallback_iiwas_vs_reached(const std_msgs::Bool::ConstPtr& msg){
    iiwas_vs_reached = (bool) msg->data;
}

//stitch_complete
void posCallback_suture_complete(const std_msgs::Bool::ConstPtr& msg){
    suture_complete = (bool) msg->data;
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

char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}

int main(int argc, char **argv) {
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "kuka_control");
    ros::NodeHandle nh;

        ros::Publisher pub_video_record_command = nh.advertise<std_msgs::Int8>("video_record_command", 100,true);
		//ros::Publisher pub3 = nh.advertise<sensor_msgs::JointState>("kuka0_command", 100);
		//ros::Publisher pub4 = nh.advertise<sensor_msgs::JointState>("kuka1_command", 100);
        ros::Publisher pub_iiwa0_destJoints = nh.advertise<sensor_msgs::JointState>("iiwa0_destJoints", 100,true);
		//ros::Publisher pub_iiwa1_destJoints = nh.advertise<sensor_msgs::JointState>("iiwa1_destJoints", 100);		
        ros::Publisher pub_needle_driver = nh.advertise<std_msgs::Bool>("needle_driver_command", 100,true);
		ros::Publisher pub_mandrel_posrodr = nh.advertise<std_msgs::Float64MultiArray>("mandrel_posrodr", 100);
        ros::Publisher pub_toolmandrel_traj= nh.advertise<std_msgs::String>("fname_traj_toolmandrel", 100,true);
        ros::Publisher pub_suture_device = nh.advertise<std_msgs::Bool>("suture_device_command", 100,true);

		//ros::Subscriber sub1 = nh.subscribe("tools_pose", 1000, posCallback1);
		ros::Subscriber sub2 = nh.subscribe("read_suture_device_angle", 1000, posCallback2);
		//ros::Subscriber sub3 = nh.subscribe("kuka0_pose", 1000, posCallback3);
		//ros::Subscriber sub4 = nh.subscribe("kuka1_pose", 1000, posCallback4);
		//ros::Subscriber sub5 = nh.subscribe("kuka0_pose_done", 1000, posCallback5);
		//ros::Subscriber sub6 = nh.subscribe("kuka1_pose_done", 1000, posCallback6);
		ros::Subscriber sub_iiwa0_reached = nh.subscribe("iiwa0_reached", 1000, posCallback_iiwa0_reached );
		ros::Subscriber sub_iiwa1_reached = nh.subscribe("iiwa1_reached", 1000, posCallback_iiwa1_reached );
		ros::Subscriber sub_force_sensor = nh.subscribe("read_mandrel_force", 1000, posCallback_read_force);
        ros::Subscriber sub_iiwas_vs_reached = nh.subscribe("iiwas_vs_reached", 1000, posCallback_iiwas_vs_reached);
        ros::Subscriber sub_suture_complete = nh.subscribe("suture_complete", 1000, posCallback_suture_complete);

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
		

		ifstream sewingSlot("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/kuka_control/src/sewing_slot.txt");
		std_msgs::String msg_fname_traj_toolmandrel;

        int list_size = 2;
//        string fname_traj_toolmandrel_list[list_size] = {
//            "toolmandrel_2018-04-19-16-03-46_s.txt_inserted_test_smooth_quat_301_900_1",
//            "toolmandrel_2018-04-19-16-03-46_s.txt_inserted_test_smooth_quat_301_900_2",
//            "toolmandrel_2018-04-19-16-03-46_s.txt_inserted_test_smooth_quat_301_900_3",
//        };
        string fname_traj_toolmandrel_list[list_size] = {
            "toolmandrel_2018-07-24-19-35-25.txt_smooth_quat_1",
            "toolmandrel_2018-07-24-19-35-25.txt_smooth_quat_2",
        };
        int traj_toolmandrel_list_count = 0;

        //msg_fname_traj_toolmandrel.data = "toolmandrel_2018-04-19-16-03-46_s.txt_inserted_test_smooth_quat_301_900_2";
        msg_fname_traj_toolmandrel.data = fname_traj_toolmandrel_list[traj_toolmandrel_list_count];
		pub_toolmandrel_traj.publish(msg_fname_traj_toolmandrel);
			
				
		if (mode==0){

				while(ros::ok()) {
					ros::spinOnce();

                    if (kbhit()){
                        //int key = cin.get();
                        int key = getch();
                        if (key==int('r')||key==int('s')||key==int('1')||key==int('2')){
                            std_msgs::Int8 msg;
                            msg.data = key;
                            cout<<"key pressed is "<<int(msg.data)<<endl;
                            pub_video_record_command.publish(msg);
                        }
                        //////////////////////////////////////////
                        else if (key==int('d')||key==int('3')){
                            std_msgs::Bool run_stitch_msg;
                            run_stitch_msg.data = true;
                            cout<<"run single stitch"<<endl;
                            pub_suture_device.publish(run_stitch_msg);

                            std_msgs::Int8 key_msg;
                            key_msg.data = int('d');
                            pub_video_record_command.publish(key_msg);
                            rate.sleep();
                        }

                    }
					
//                    //send posrodr to mandrel
//					double posrodr;
//					std_msgs::Float64MultiArray msg_posrodr;
//					for (int i = 0; i <6; i ++){
//						sewingSlot>>posrodr;
//						msg_posrodr.data.push_back(posrodr);
//					}
/*
						if (count0<jointsPose.size()){
							//if (kuka0_reached && kuka1_reached){
							if (kuka0_reached){
								sensor_msgs::JointState msg;	
								for (int i = 0; i <7; i ++){
										msg.name.push_back("iiwa0 joint "+to_string(i));
										msg.position.push_back(jointsPose[count0][i]);
										cout<<jointsPose[count0][i]<<" ";
								}
								cout<<endl;
								rate.sleep();
								pub_iiwa0_destJoints.publish(msg);
								kuka0_reached = false;
								count0++;
							}
						}

*/
/*
						if (count0<jointsPose.size()){
							//if (kuka0_reached && kuka1_reached){
							//if (kuka1_reached){
									sensor_msgs::JointState msg;	
									for (int i = 0; i <7; i ++){
											msg.name.push_back("iiwa1 joint "+to_string(i));
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
                    if(iiwas_vs_reached){
                        cout<< "iiwas_vs_reached == true"<<endl;

                        traj_toolmandrel_list_count = (traj_toolmandrel_list_count+1)%list_size;

                        //if traj_toolmandrel_list_count == 1, run single suture
                        if (traj_toolmandrel_list_count == 1){
                            std_msgs::Bool run_suture;
                            run_suture.data = true;
                            pub_suture_device.publish(run_suture);
                            //wait for suture to finish
                            while(!suture_complete){
                                ros::spinOnce();
                            }
                            suture_complete = false;
                        }
                        iiwas_vs_reached = false;

                        //rotate mandrel to the next sewing slot
                        std_msgs::Float64MultiArray msg_mandrel;
                        for(int i = 0; i < 6; i ++){
                            double tmp;
                            sewingSlot >> tmp;
                            msg_mandrel.data.push_back(tmp);
                        }
                        pub_mandrel_posrodr.publish(msg_mandrel);

//                        //run needle driver
//                        pub_needle_driver.publish(true);

                        //publish new trajectory to iiwa_vs_core
                        std_msgs::String msg_fname_traj_toolmandrel;
                        msg_fname_traj_toolmandrel.data = fname_traj_toolmandrel_list[traj_toolmandrel_list_count];
                        pub_toolmandrel_traj.publish(msg_fname_traj_toolmandrel);




                    }

				}
    
		

		}
        if (mode == 1){
            /*
            //generate trajectory and read trajectory
            //system("python /home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/scripts/example_aico_trajectory.py 1");

            ifstream infile("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/traj_solution_smooth_r002");
            //ifstream infile("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/overall_traj_bk_201806081723.txt");
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
*/

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
        }

  return 0;

}
