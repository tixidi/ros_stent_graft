#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include <fstream>
#include <iostream>
#include <ctime>
#include <chrono>
using namespace std;
using namespace std::chrono;

double iiwa_msrTransform[12];
ofstream outfile_kuka_traj;


void posCallback_iiwa_msrTransform(const std_msgs::Float64MultiArray::ConstPtr& msg){
	int64_t nowms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	outfile_kuka_traj<<nowms<<" ";
	for (int j = 0; j <12; j ++){
		outfile_kuka_traj<<msg->data[j]<<" ";
	}
	outfile_kuka_traj<<endl;

}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "test_kuka_tf_subscribe");
    ros::NodeHandle nh;
	outfile_kuka_traj.open("kuka_tf_received.txt");

	ros::Subscriber sub_iiwa_msrTransform = nh.subscribe("iiwa0_msrTransform", 1000, posCallback_iiwa_msrTransform);
	

	ros::spin();

    return 0;
}

