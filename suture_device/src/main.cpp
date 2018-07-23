#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h> 
#include <iostream>

#include "Faulharbermotor.h"
#include "footPadel.h"

using namespace std;

Faulharbermotor *sutureDevice;
footPadel_philip *footPadel;

void posCallback(const std_msgs::Bool::ConstPtr& msg)
{
	cout <<"msg->data " << msg->data << endl;
	if(msg->data){
		cout<<"run stitch"<<endl;
		//to do: run single stitch
		sutureDevice->runSingleStitch();
	}

}

int main(int argc, char *argv[])
{
		//Initialise Publisher and Subscriber
		ros::init(argc, argv, "suture_device");
    ros::NodeHandle nh;
    ros::Publisher pub=nh.advertise<std_msgs::Float32>("read_suture_device_angle", 100);
		ros::Subscriber sub = nh.subscribe("suture_device_command", 1000, posCallback);
    srand(time(0));
    ros::Rate rate(10);


		//Initialise Suture Device
		sutureDevice = new Faulharbermotor();
		footPadel = new footPadel_philip();
		footPadel->start();
		//sutureDevice->enableToggle();
		
		
		while(ros::ok()) {
			ros::spinOnce();
			std_msgs::Float32 msg;
			deviceInfomation disInfo= sutureDevice->getAllCtrlInfomation();
			msg.data = disInfo.msrPos[0];
			pub.publish(msg);
			
			rate.sleep();
		}
		

		
    return 0;
}
