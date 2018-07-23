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
    if (argc<3 || argc>3) {
        cerr<<"Invalid number of arguments"<<endl;
        cerr<<"Usage: [kuka index 0:left 1:right] [mode 0:kuka 1:test]"<<endl;
        return 0;
    }
    if (int(atoi(argv[1]))>1 ||int(atoi(argv[1]))<0 ){
            cerr<<"Invalid kuka index [0:left 1:right]"<<endl;
            return 0;
    }
    if (int(atoi(argv[2]))>1 ||int(atoi(argv[2]))<0 ){
            cerr<<"Invalid mode [0:kuka 1:test]"<<endl;
            return 1;
    }
    QApplication *app=new QApplication(argc, argv);

    pathPlanner=new KukaMotionPlanning();
    pathPlanner->RUNROBOT_index_sub = std::stoi(argv[1]);
    pathPlanner->RUNROBOT_mode = int(atoi(argv[2])); // 0:kuka mode 1:test mode

    cout << "RUNROBOT_index_sub " << pathPlanner->RUNROBOT_index_sub << endl;
    cout << "RUNROBOT_mode " << pathPlanner->RUNROBOT_mode << endl;

    pathPlanner->start(QThread::HighPriority);
    cout << " pathPlanner->start(QThread::HighPriority);!" << endl;

    app->exec();

    return 0;
    //TODO:
    // publish iiwa reach = true
    // publish iiwa joint = current joint
    //TODO: handle recorded files
}
