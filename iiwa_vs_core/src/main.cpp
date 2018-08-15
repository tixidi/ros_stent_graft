#include <QApplication>
#include "GUI/mainWindow.h"
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include "sensor_msgs/JointState.h"
#include <iiwa_test/iiwaState.h>
#include <map>

#include "VisionSystem/globals.h"
#include "kukaMotionPlanning.h"

//int   main(int argc, char *argv[])
//{
//    QApplication *app=new QApplication(argc, argv);
//    //initiate whole applicaiton
//    mainWindow * kukaContorlApp= new mainWindow();

//    kukaContorlApp->show();
//    return app->exec();
//    ros::spin();

//}


KukaMotionPlanning *pathPlanner;

int   main(int argc, char **argv)
{

    QApplication *app=new QApplication(argc, argv);

    pathPlanner=new KukaMotionPlanning();
    pathPlanner->argc = argc;
    pathPlanner->argv = argv;

    pathPlanner->start(QThread::HighPriority);
    cout << " pathPlanner->start(QThread::HighPriority);!" << endl;

    app->exec();

    return 0;
    //TODO:
    // publish iiwa reach = true
    // publish iiwa joint = current joint
    //TODO: handle recorded files
}
