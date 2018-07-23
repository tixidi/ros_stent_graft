//**************************************************************
// kinematic control layer, inverse kinematics implemented here
// input : cateasion pos and ori
// output: desire joint values
//*************************************************************
#ifndef KUKAKINEMATICCONTROL_H
#define KUKAKINEMATICCONTROL_H
#include <QThread>
#include <QTime>
#include "functions.h"
#include "kukaKinematicModel.h"
#include <fstream>

#define SAMPLETIME 1 //ms

class KukaKinematicControl: public QThread
{
    Q_OBJECT
public :
      KukaKinematicControl(int port,const char *kukaIPAddress,const char *localIPAddress, Matrix4d  &coord);
      void          setDesTipPosture(robotPosture  &PosQua);
      bool          moveToPostureInTipFrame(robotPosture  &desPosQua);
      bool          moveToPosture(robotPosture  &PosQua);
      robotPosture  getCurrTipPosture();
      Vector7d      getJointAngles();
      kukaKinematicModel* getRobotInstance();
      bool ArmReady();
      void robotStop();
      void robotConnect();
      void robotDisconnect();
      void setJointAngles(Vector7d &Joints);

private :     
      kukaKinematicModel *kukaRobot;
      //for critical timing
      QTime *timeCnt;
      int prevTime;
      Vector7d desJointPos;
      bool firstTime;

      void operaSpaceControl();
      void JointSpaceControl();
      Vector7d calInverseKinematic();
      void run();

      robotPosture  desPosQuaKuka;
      bool Ready;
      int robotReference;
      MatrixXd weight;

      //for move to
      robotPosture  iniMovPosture;
      bool reachTarget;
      int stepCnt;
      int interpoInterval;

      std::ofstream f_inter;

      robotPosture currTipPosture;
      bool reachTargetTip;
      Quaterniond preq;


};

#endif
