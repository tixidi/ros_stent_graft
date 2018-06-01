#include <QApplication>
#include "GUI/mainWindow.h"
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

ros::Publisher pub_iiwa0_desiredEEInRob;
//ros::Rate rate(10);
int   main(int argc, char *argv[])
{
		//initialise ros and setup nodes
    ros::init(argc,argv,"iiwa_visual_servoing");	
		ros::NodeHandle nh;
		pub_iiwa0_desiredEEInRob = nh.advertise<std_msgs::Float64MultiArray>("iiwa0_desiredEEInRob", 100);
		//pub_iiwa1_desiredEEInRob = nh.advertise<std_msgs::Float64MultiArray>("iiwa1_desiredEEInRob", 100);
		srand(time(0));
		//ros::Rate rate(10);
		

    QApplication *app=new QApplication(argc, argv);
    //initiate whole applicaiton
    mainWindow * kukaContorlApp= new mainWindow();
    kukaContorlApp->show();

    return app->exec();

}

