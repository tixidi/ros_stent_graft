#include <ros/ros.h>
//#include "std_msgs/String.h"
#include "NeedleDriverInterface.h"
#include <geometry_msgs/Wrench.h> 
#include <std_msgs/Bool.h> 
#include <iostream>
#include <QApplication>
using namespace std;


int pos = 0;
NeedleDriverInterface* needleDriver;
//void posCallback(const geometry_msgs::Wrench::ConstPtr& msg)
//{

//	cout<<"force "<<msg->force.x<<endl;
//	if (msg->force.x>=0){
//		pos+=500;
//	}

//	if (msg->force.x<0){
//		pos-=500;
//	}



//	if (pos>4000){
//		pos = 4000;
//	}
//	if (pos<0){
//		pos = 0;
//	}
//	std::cout<<pos<<endl;
//	needleDriver->changePos0(pos);


//	//usleep(1000000);


////  ROS_INFO("I heard: [%s]", msg->data.c_str());
////std::cout<<"x: "<<msg->getX()<<" y: "<<msg.getY()<<" z: "<<msg.getZ()<<std::endl;
////std::cout<<msg->force<<endl;

//}

void posCallback_needle_driver(const std_msgs::Bool::ConstPtr& msg)
{
    if ((bool)msg->data)
        needleDriver->changePos0(4000);
    if (!(bool)msg->data)
        needleDriver->changePos0(0);
}

int main(int argc, char **argv) {
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "needle_driver");
    ros::NodeHandle nh;
    QApplication app(argc, argv);
    needleDriver = new NeedleDriverInterface();
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called posCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
//    ros::Subscriber sub = nh.subscribe("read_mandrel_force", 1000, posCallback);
    ros::Subscriber sub_needle_driver = nh.subscribe("needle_driver_command", 1000, posCallback_needle_driver);
    int count  = 0;

    /*while(true){
		needleDriver->changePos0(pos);
        usleep(5000000);
		if (count%2==0)
			pos+=2000;
		else
			pos-=2000;
		count++;
		cout<<"pos = "<<pos<<endl;
		
    }*/


  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;

}
