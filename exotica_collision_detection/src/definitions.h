#ifndef DEFINITION_H
#define DEFINITION_H

#include <Eigen/Dense>
#include <Eigen/Core>
#include <math.h>
#include <QDebug> //QT output
#include <iostream>
//#include "config.h"
//#define PI 3.141592653
using namespace Eigen;
using namespace std;

typedef Eigen::Matrix<double, 4,4> Matrix4d;
typedef Eigen::Matrix<double, 6,7> Matrix67d;
typedef Eigen::Matrix<double, 7,1> Vector7d;
typedef Eigen::Vector3d            Vector3d;
typedef Eigen::Matrix<double, 7,6> Matrix76d;
typedef Eigen::Matrix<double, 6,1> Vector6d;
typedef Eigen::Matrix<double, 3,3> Matrix3d;
typedef Eigen::Matrix<double, 7,7> Matrix7d;
typedef Eigen::Matrix<double, 6,6> Matrix6d;
typedef Eigen::MatrixXf  MatrixXf;
typedef Eigen::Matrix<double, 6,19> Matrix196d;
typedef Eigen::Vector4d             Vector4d;
typedef Eigen::Matrix<double, 14,1> Vector14d;
typedef Eigen::Matrix<double, 12,1> Vector12d;
typedef Eigen::Quaterniond        Quaterniond;
typedef Eigen::Matrix<double, 6,14> Matrix614d;
typedef Eigen::Matrix<double, 14,6> Matrix146d;
typedef Eigen::Matrix<double, 14,14> Matrix1414d;

class robotPosture : public Matrix4d
{
private:
    Quaterniond qua;
    Vector3d pos;
public:
    robotPosture ()
    {
        qua=Quaterniond (Matrix3d::Identity());
        pos<<0,0,0;
    }
    void setQuaternion(Quaterniond &q )
    {
        qua=q;
    }
    void setPosition(Vector3d  &v)
    {
        pos=v;
    }
    void setPosition(double a, double b, double c)
    {
        pos(0)=a;
        pos(1)=b;
        pos(2)=c;
    }

    void setQuaternion(double w, double x, double y, double z)
    {
        qua.w()=w;
        qua.x()=x;
        qua.y()=y;
        qua.z()=z;
    }

    Quaterniond & getQuaternion()
    {
        return qua;
    }
    Vector3d& getPosition()
    {
        return pos;
    }   
//    void translate(Vector3d &tran )
//    {
//        pos(0)=pos(0)+tran(0);
//        pos(1)=pos(1)+tran(1);
//        pos(2)=pos(2)+tran(2);
//    }
//    void translate (double a,double b, double c)
//    {
//        pos(0)=pos(0)+a;
//        pos(1)=pos(1)+b;
//        pos(2)=pos(2)+c;
//    }
//    void rotate (Quaterniond &rot )
//    {
//        qua=rot*qua;
//    }

//    void rotateAroundXAxisDegree(double a)
//    {
//         Vector3d xAxis(1,0,0);
//         AngleAxisd rotate(a*PI/180, xAxis);
//         Quaterniond quaTemp(rotate);
//         qua=quaTemp*qua;
//    }
//    void rotateAroundYAxisDegree(double a)
//    {
//        Vector3d YAxis(0,1,0);
//        AngleAxisd rotate(a*PI/180, YAxis);
//        Quaterniond quaTemp(rotate);
//        qua=quaTemp*qua;
//    }
//    void rotateAroundZAxisDegree(double a)
//    {
//        Vector3d zAxis(0,0,1);
//        AngleAxisd rotate(a*PI/180, zAxis);
//        Quaterniond quaTemp(rotate);
//        qua=quaTemp*qua;
//    }

//    void transformInBaseFrame(robotPosture &a)
//    {
//        pos(0)=pos(0)+a.getPosition()(0);
//        pos(1)=pos(1)+a.getPosition()(1);
//        pos(2)=pos(2)+a.getPosition()(2);
//        qua=a.getQuaternion()*qua;
//    }

//    void transformInTipFrame(robotPosture &a)
//    {
//        qua=qua*a.getQuaternion();
//        Matrix3d mat(qua);
//        pos= mat*a.getPosition() +pos;
//    }

//    void translateInBaseFrame (Vector3d &tran)
//    {
//        pos(0)=pos(0)+tran(0);
//        pos(1)=pos(1)+tran(1);
//        pos(2)=pos(2)+tran(2);
//    }
//    void translateInTipFrame(Vector3d &tran)
//    {
//        Matrix3d mat(qua);
//        pos= mat*tran +pos;
//    }

//    void rotateInBaseFrame (Quaterniond &rot)
//    {
//        qua=rot*qua;
//    }
//    void rotateInTipFrame(Quaterniond &rot)
//    {
//        qua=qua*rot;
//    }
//    void rotateAroundBaseXAxisDegree(double a)
//    {
//         Vector3d xAxis(1,0,0);
//         AngleAxisd rotate(a*PI/180, xAxis);
//         Quaterniond quaTemp(rotate);
//         qua=quaTemp*qua;
//    }
//    void rotateAroundBaseYAxisDegree(double a)
//    {
//        Vector3d YAxis(0,1,0);
//        AngleAxisd rotate(a*PI/180, YAxis);
//        Quaterniond quaTemp(rotate);
//        qua=quaTemp*qua;
//    }
//    void rotateAroundBaseZAxisDegree(double a)
//    {
//        Vector3d zAxis(0,0,1);
//        AngleAxisd rotate(a*PI/180, zAxis);
//        Quaterniond quaTemp(rotate);
//        qua=quaTemp*qua;
//    }
//    void rotateAroundTipXAxisDegree(double a)
//    {
//        Vector3d xAxis(1,0,0);
//        AngleAxisd rotate(a*PI/180, xAxis);
//        Quaterniond quaTemp(rotate);
//        qua=qua*quaTemp;
//    }
//    void rotateAroundTipYAxisDegree(double a)
//    {
//        Vector3d YAxis(0,1,0);
//        AngleAxisd rotate(a*PI/180, YAxis);
//        Quaterniond quaTemp(rotate);
//        qua=qua*quaTemp;
//    }
//    void rotateAroundTipZAxisDegree(double a)
//    {
//        Vector3d zAxis(0,0,1);
//        AngleAxisd rotate(a*PI/180, zAxis);
//        Quaterniond quaTemp(rotate);
//        qua=qua*quaTemp;
//    }

};

class kukaSVD
{
public:
    Matrix6d U;
    Vector6d sValue;
    Matrix7d V;
};

//#ifndef kukaInformation
class kukaInformation
{
public:
    Vector7d  jointAngles1;
//    Vector6d  cartPosOri1;
    Matrix4d  gloHomoMatrix1;
    Vector7d  jointAngles2;
//    Vector6d  cartPosOri2;
    Matrix4d  gloHomoMatrix2;
    robotPosture kukaPose0;
    robotPosture kukaPose1;
} ;

#endif
