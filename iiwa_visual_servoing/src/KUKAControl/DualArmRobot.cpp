#include "DualArmRobot.h"
#include <QDebug>
extern double bb;
extern double aa;

//double aa = 31.6;
//double bb = 49.6;
DualArmRobot::DualArmRobot()
{
    Matrix4d  coord1=Matrix4d::Identity();
    Matrix4d  coord2=Matrix4d::Identity();
    coord2(0,3)=-aa*0.025;
    coord2(1,3)= bb*0.025;
    coord2(2,3)= 0.0;

    kukaRobot[0]= new KukaKinematicControl(49938,"192.168.0.20","192.168.0.100",coord1 );//kuka0
    kukaRobot[1]= new KukaKinematicControl(49939,"192.168.1.20","192.168.1.100",coord2 );//kuka1


    //Bidan set kuka1 initial joints
    Vector7d kuka1_init;
    kuka1_init << -111*3.1415926/180, (40-90)*3.1415926/180, 38*3.1415926/180, 58*3.1415926/180,60*3.1415926/180 ,-58*3.1415926/180, -36*3.1415926/180;
    kukaRobot[1]->setJointAngles(kuka1_init);

    kukaRobot[0]->start(QThread::TimeCriticalPriority);
    kukaRobot[1]->start(QThread::TimeCriticalPriority);
}

bool DualArmRobot::dualArmReady()
{
    if ( kukaRobot[0]->ArmReady() ||  kukaRobot[1]->ArmReady() )
    return true;
    else
    return false;
}

void DualArmRobot::setDesTipPosture(robotPosture  &PosQua, int ref)
{
   if (ref==0)
    kukaRobot[0]->setDesTipPosture(PosQua);
   else
    kukaRobot[1]->setDesTipPosture(PosQua);
}

bool DualArmRobot::moveToPosture(robotPosture  &PosQua, int ref)
{
    if (ref==0)
    {
        return kukaRobot[0]->moveToPosture(PosQua);
    }
    else
    {
        return kukaRobot[1]->moveToPosture(PosQua);
    }
}

bool DualArmRobot::moveToPostureInTipFrame(robotPosture  &PosQua, int ref)
{
    if (ref==0)
    {
        return kukaRobot[0]->moveToPostureInTipFrame(PosQua);
    }
    else
    {
        return kukaRobot[1]->moveToPostureInTipFrame(PosQua);
    }
}

robotPosture  DualArmRobot::getCurrTipPosture(int refernce)
{
    if (refernce==0)
        return kukaRobot[0]->getCurrTipPosture();
    else if (refernce==1)
        return kukaRobot[1]->getCurrTipPosture();
}

Vector7d DualArmRobot::getJointAngles(int refernce)
{
    if (refernce==0)
        return kukaRobot[0]->getJointAngles();
    else if (refernce==1)
        return kukaRobot[1]->getJointAngles();
}

kukaKinematicModel* DualArmRobot::getRobotInstance(int refernce)
{
    if (refernce==0)
        return kukaRobot[0]->getRobotInstance();
    else if (refernce==1)
        return kukaRobot[1]->getRobotInstance();
}

void DualArmRobot::dualrobotStop()
{
    kukaRobot[0]->robotStop();
    kukaRobot[1]->robotStop();
}
void DualArmRobot::dualrobotConnect()
{
    kukaRobot[0]->robotConnect();
    kukaRobot[1]->robotConnect();
}
void DualArmRobot::dualrobotDisconnect()
{
    kukaRobot[0]->robotDisconnect();
    kukaRobot[1]->robotDisconnect();
}

