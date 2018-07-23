#include <ros/ros.h>
#include <iostream>
#include "omd/opto.h"
#include <unistd.h>
#include <geometry_msgs/Wrench.h> 
#include <stdlib.h> 
#include "forceSensor.h"

using namespace std;

int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
     ros::init(argc, argv, "force_sensor");
     ros::NodeHandle nh;

     //Ceates the publisher, and tells it to publish
     //to the stentgraft/read_force topic, with a queue size of 100
     ros::Publisher pub=nh.advertise<geometry_msgs::Wrench>("read_mandrel_force", 100, true);

     //Sets up the random number generator
     srand(time(0));

     //Sets the loop to publish at a rate of 10Hz
     ros::Rate rate(10);

     //Initialise force sensor
     cout << "Initialising force sensor" << endl;
     forceSensor fsensor;

       while(ros::ok()) {
	     std::cout<<"x: "<<fsensor.getX()<<" y: "<<fsensor.getY()<<" z: "<<fsensor.getZ()<<std::endl;
            //Declares the message to be sent
            geometry_msgs::Wrench msg;
            msg.force.x = fsensor.getX();
            msg.force.y = fsensor.getY();
            msg.force.z = fsensor.getZ();
           //Publish the message
           pub.publish(msg);

          //Delays untill it is time to send another message
          rate.sleep();
         }
}



