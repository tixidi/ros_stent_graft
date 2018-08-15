#include <ros/ros.h>
#include "mandrelDriverInterface.h"
#include <std_msgs/Float64MultiArray.h> 
#include <std_msgs/Bool.h>.h>
#include <iostream>
#include <QApplication>
using namespace std;

double prev_posrodr[6] = {0,0,0,0,0,0};
double curr_posrodr[6] = {0,0,0,0,0,0};
MandrelDriverInterface* mandrelDriver;
ros::Publisher pub_mandrel_complete;
int pos = 0;

void posCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{

	
    for (int i = 0; i < 6; i++){
		prev_posrodr[i] = curr_posrodr[i];
		curr_posrodr[i] = msg->data[i];
	}
	

	//pos must be between 0 and 4095
    while (curr_posrodr[3]<0){
        curr_posrodr[3] = curr_posrodr[3]+3.1415926*2;
    }
    while (curr_posrodr[3]>3.1415926*2){
        curr_posrodr[3] = curr_posrodr[3]-3.1415926*2;
    }
    pos = curr_posrodr[3]/(3.1415926*2)*4095;
    cout<<pos<<endl;
    mandrelDriver->changePos(pos);


}

void posCallback_mandrel_command(const std_msgs::Bool::ConstPtr& msg)
{
    pos+=4095/12;
    mandrelDriver->changePos(pos);
    std_msgs::Bool msg_complete;
    msg_complete.data = true;
    pub_mandrel_complete.publish(msg_complete);
}


int main(int argc, char **argv) {
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "mandrel_device");
    ros::NodeHandle nh;
    QApplication app(argc, argv);
    mandrelDriver = new MandrelDriverInterface();

    //ros::Subscriber sub = nh.subscribe("mandrel_posrodr", 1000, posCallback);
    pub_mandrel_complete = nh.advertise<std_msgs::Bool>("mandrel_complete", 100,true);
    ros::Subscriber sub_mandrel_command = nh.subscribe("mandrel_command", 1000, posCallback_mandrel_command);
    int count  = 0;
	
//    while(true){
//        mandrelDriver->changePos(0);
//		usleep(1000000);

		
//		cout<<"pos = "<<pos<<endl;
//        pos+=512;

//        if (pos>4096)
//            pos-=4095;
		
//    }



  ros::spin();

  return 0;

}
