//**************************************************************
// motion planning layer, inverse kinematics implemented here
// input : cateasion pos and ori
// output: desire joint values
//*************************************************************
#ifndef IIWACONTROL_H
#define IIWACONTROL_H

//#include "KUKAControl/definitions.h"
#include <QThread>
#include <QTime>
#include <QTimer>

#include <kukasunrise.h>
#include <kukasunriseplanned.h>

#define SAMPLETIME 2 //ms


class iiwaControl: public QThread
{
    Q_OBJECT
public :
    KukaSunrisePlanned iiwa;
    KukaSunrisePlanned::InitialParameters params;
    iiwaControl(int robotID);
    Erl::Transformd getiiwaPose();
    void setiiwaPose(Erl::Transformd &desiredPose);
    void EnableGravityCompensation();
    void DisableGravityCompensation();
    bool iiwaReached();

    bool iiwaConnected;
private :     
      void run();
      Erl::Transformd iiwaPose;
};


#endif
