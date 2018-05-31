//**************************************************************
// motion planning layer, inverse kinematics implemented here
// input : cateasion pos and ori
// output: desire joint values
//*************************************************************
#ifndef FORCESENSOR_H
#define FORCESENSOR_H

//#include <QThread>
//#include <QTime>
//#include <QTimer>

#include "omd/opto.h"
#include <unistd.h>
#include <iostream>


class forceSensor//: public QThread
{
    //Q_OBJECT
public :
    forceSensor();
    int getX();
    int getY();
    int getZ();
    void zeroSensor();
    int fs_running;

private :     
      void run();
      OptoDAQ daq;
      OptoPorts ports;
      OptoPackage pack3D;
};

#endif
