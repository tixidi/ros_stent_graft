#include <ros/ros.h>
#include "mandrelDriverInterface.h"
#include <std_msgs/Float64MultiArray.h> 
#include <iostream>
#include <QApplication>
using namespace std;

double prev_posrodr[6] = {0,0,0,0,0,0};
double curr_posrodr[6] = {0,0,0,0,0,0};
MandrelDriverInterface* mandrelDriver;
double pos = 0;
void posCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{

	
	/*for (int i = 0; i < 6; i++){
		prev_posrodr[i] = curr_posrodr[i];
		curr_posrodr[i] = msg->data[i];
	}
	
	cout<<pos<<endl;
	//pos must be between 0 and 4095
	pos = curr_posrodr[3]/3.1415926*4095;
	//mandrelDriver->changePos(pos);
*/
}




int main(int argc, char **argv) {
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "mandrel_device");
    ros::NodeHandle nh;
    QApplication app(argc, argv);
    mandrelDriver = new MandrelDriverInterface();

    ros::Subscriber sub = nh.subscribe("mandrel_posrodr", 1000, posCallback);
	
    /*int count  = 0;
	
    while(true){
		mandrelDriver->changePos(pos);
		usleep(1000000);
			
		
		cout<<"pos = "<<pos<<endl;
		pos+=512;
		
	}*/
	

  ros::spin();

  return 0;

}
