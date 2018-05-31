#ifndef kukaKinematicModel_H
#define kukaKinematicModel_H
#include <QTime>
#include <QThread>
#include "FRI/friremote.h"
#include "functions.h"

class kukaKinematicModel: public QThread
{
    Q_OBJECT
public:
    kukaKinematicModel(int port,const char *kukaIPAddress,const char *localIPAddress, Matrix4d  &coord);
    void    setJointAngles(Vector7d &joint);
    bool    moveToPosture(robotPosture  &PosQua);
    void    robotStop();
    void    robotDisconnect();
    void    robotConnect();
    bool    robotReady();
    int     robotState();
    Matrix4d getGlobalTip();
    Matrix4d getGlobalBase();

    Quaterniond getQuaternion();
    Matrix3d    getRotMatrix()   ;
    Vector3d    getCartPosition() ;
    Vector7d    getJointAngles();
    MatrixXd    getJacobian();
    robotPosture  getRobtPosture();
    Matrix4d    getHomoMatrix();
    Matrix4d    gloCoor;
protected:
    friRemote   *kukaRobot;
    int         statusVariable;
    bool        running;
    bool        init;
    bool        startfirst;
    Vector7d    previousJoints;
    int robotReference;

    struct MeasuredData
    {
        Matrix67d Jacobian;
        Vector7d  jointAngles;
        Vector3d  cartPosition;
        Matrix3d  rotMatrix;
    }kukaMsr;

    struct CommandData
    {
        Vector7d  jointAngles;
    }kukaCmd;

    //correct quaternion err
    Quaterniond preq;
    int sign;

    Matrix67d calcuJacobin(Vector7d &jointAngle);
    Matrix4d forwardKinematics(Vector7d &joint);
    void SendToKuka();
    void readFromKuka();
    void run();

    QTime *timeCnt;
    int prevTime;

    Matrix3d rotMatrix(double jointAngle, int i);
    Matrix4d hoMatrix (double jointAngle, int i)  ;
    Matrix4d Rx(double tempAfa);
    Matrix4d Dx(double tempA);
    Matrix4d Rz(double tempTheta);
    Matrix4d Qz(double tempD);  
    //D-H parameters
    Vector7d kukaD;
    Vector7d kukaA;
    Vector7d kukaAfa;
};

#endif
