#include <ros/ros.h>
#include "mandrelDriverInterface.h"
#include <std_msgs/Float64MultiArray.h> 
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
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
    //need to fix
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
    int slot_num = 12;
    pos+=4095/slot_num;
    mandrelDriver->changePos(pos);
    std_msgs::Bool msg_complete;
    msg_complete.data = true;
    pub_mandrel_complete.publish(msg_complete);
}

void posCallback_mandrel_degree(const std_msgs::Float64& msg)
{
    pos += int(4095/360 * msg.data);
    if (pos<= 4095 && pos >= 0)
        mandrelDriver->changePos(pos);
    else
        pos -= int(4095/360 * msg.data);
    cout << "Mandrel set to " << pos << endl;

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
    ros::Subscriber sub_mandrel_degree = nh.subscribe("mandrel_degree", 1000, posCallback_mandrel_degree);
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
