#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <iostream>

#include "Faulharbermotor.h"
#include "footPadel.h"

using namespace std;

Faulharbermotor *sutureDevice;
footPadel_philip *footPadel;
ros::Publisher pub_suture_complete;
ros::Subscriber sub_suture_device_command;

void posCallback(const std_msgs::Bool::ConstPtr& msg)
{
    cout <<"msg->data " << (bool)msg->data << endl;
    bool runSingleSuture = (bool)msg->data;
    if (runSingleSuture){
        deviceInfomation disInfo = sutureDevice->getAllCtrlInfomation();
        cout<<"run stitch"<<endl;
        sutureDevice->runSingleStitch();
        runSingleSuture = false;
        if (disInfo.status == 0){
            cout<<"stitch complete"<<endl;
            std_msgs::Bool msg_complete;
            msg_complete.data = true;
            pub_suture_complete.publish(msg_complete);
        }
    }

}

int main(int argc, char *argv[])
{
    //Initialise Publisher and Subscriber
    ros::init(argc, argv, "suture_device");
    ros::NodeHandle nh;
    pub_suture_complete = nh.advertise<std_msgs::Bool>("suture_complete", 100);
    sub_suture_device_command = nh.subscribe("suture_device_command", 1000, posCallback);
    srand(time(0));
    ros::Rate rate(10);


    //Initialise Suture Device
    sutureDevice = new Faulharbermotor();
    footPadel = new footPadel_philip();
    footPadel->start();
    //sutureDevice->enableToggle();

    ros::spin();

		
    return 0;
}
