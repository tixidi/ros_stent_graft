#include <QApplication>
#include "GUI/mainWindow.h"
#include <ros/ros.h>

int   main(int argc, char *argv[])
{
    QApplication *app=new QApplication(argc, argv);
    //initiate whole applicaiton
    mainWindow * kukaContorlApp= new mainWindow();

    kukaContorlApp->show();


    return app->exec();
    ros::spin();

}

