//**************************************************************
// Dual arm fuse layer
//*************************************************************
#ifndef DUALARMROBOT_H
#define DUALARMROBOT_H
#include <QThread>
#include <QTime>
#include "functions.h"
#include "kukaKinematicControl.h"

class DualArmRobot: public QThread
{
    Q_OBJECT
public :
      DualArmRobot();
      void setDesTipPosture(robotPosture &PosQua, int reference=1);
      bool moveToPosture(robotPosture  &PosQua, int ref);
      bool moveToPostureInTipFrame(robotPosture  &PosQua, int ref);
      robotPosture  getCurrTipPosture(int refernce);
      Vector7d getJointAngles(int refernce=0);
      kukaKinematicModel* getRobotInstance(int refernce);
      bool dualArmReady();
      void dualrobotStop();
      void dualrobotConnect();
      void dualrobotDisconnect();
private :
      KukaKinematicControl *kukaRobot[2];
      bool dualReady;
};

#endif
