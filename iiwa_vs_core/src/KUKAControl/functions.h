#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Eigen/Dense>
#include <Eigen/Core>
#include <math.h>
#include <QDebug> //QT output
#include <iostream>
#include "definitions.h"
#include <opencv2/opencv.hpp>
#include "iiwaControl/iiwaControl.h"
#include <aruco/aruco.h>
#include <aruco/marker.h>
#include <aruco/markerdetector.h>
#include <fstream>      // std::ifstream
#include <string>

class Functions
{
public:
    static MatrixXd augumentJacobian(MatrixXd &jaco1,MatrixXd &jaco2)
    {
        int row1=jaco1.rows();
        int row2=jaco2.rows();
        int cols=jaco1.cols();

        MatrixXd augJaco(row1+row2, cols);

        for (int i=0;i<row1;i++)
            for (int j=0;j<cols;j++)
               augJaco(i,j)= jaco1(i,j);

        for (int i=row1;i<row1+row2;i++)
            for (int j=0;j<cols;j++)
               augJaco(i,j)= jaco2(i-row1,j);

        return augJaco;
    }

    static double sgn(double input)
    {
        double output=1;
        if (input>=0)
        output=1;
        else
        output=-1;
        return output;
    }

    static MatrixXd pinv(MatrixXd &jaco)
    {
        MatrixXd jacoTranspose= jaco.transpose();
        MatrixXd jacoInverse= ( jaco*jacoTranspose ).inverse();
        MatrixXd jacoPseudo =jacoTranspose*jacoInverse;
        return jacoPseudo;
    }

    static MatrixXd Wpinv (MatrixXd & jaco, MatrixXd &weight)
    {
        MatrixXd weightInv=weight.inverse();
        MatrixXd jacoT=jaco.transpose();
        MatrixXd weightedInv=weightInv* jacoT*( (jaco*weightInv*jacoT).inverse() );
        return weightedInv;
    }

    static VectorXd combinedVector(VectorXd &a, VectorXd &b)
     {
         VectorXd all=VectorXd::Zero(a.rows()+b.rows() );
         for (int i=0;i<a.rows();i++)
             all(i)=a(i);
         for (int i=0;i<b.rows();i++)
             all(i+a.rows())=b(i);
         return all;
     }  

    static Quaterniond Rot2Quaternion(Matrix4d & rot   )
    {
        Matrix3d tempRotation=MatrixXd::Zero(3,3);
        for (int i=0; i<3; i++)
            for (int j=0; j<3; j++)
                tempRotation(i,j) = rot(i,j);
        Quaterniond tempQuater (tempRotation);
        return tempQuater;
    }
    //overload for matrix 3*s
    static Quaterniond Rot2Quaternion(Matrix3d & rot   )
    {
        Matrix3d tempRotation=MatrixXd::Zero(3,3);
        for (int i=0; i<3; i++)
            for (int j=0; j<3; j++)
                tempRotation(i,j) = rot(i,j);
        Quaterniond tempQuater (tempRotation);
        return tempQuater;
    }

    static Vector3d calQuaternionErr (Quaterniond &des, Quaterniond &crr)
    {
        Vector3d err;
        Vector3d q1 (des.x(),des.y(),des.z() ) ;
        double   a1=des.w();

        Vector3d q2 (-crr.x(),-crr.y(),-crr.z());
        double   a2=crr.w();
        err= a1*q2 +a2*q1 +q1.cross(q2);
        return err;
    }
    static VectorXd linearInterpolation(Vector3d &va, Vector3d &vb, double t )
    {
        if(t<0)
        {
            return va;
        }
        if(t>1)
        {
            return vb;
        }
        else
        {
            
            VectorXd Vtemp= va + t* (vb-va);
            return Vtemp;
        }
    }

    static Quaterniond slerp(Quaterniond &qa, Quaterniond &qb, double t)
    {
        // quaternion to return
        if(t<0)
        {
            return qa;
        }
        if(t>1)
        {
            return qb;
        }
        else
        {
            Quaterniond qm ;
            // Calculate angle between them.
            double cosHalfTheta = qa.w() * qb.w() + qa.x() * qb.x() + qa.y() * qb.y() + qa.z() * qb.z();
            // if qa=qb or qa=-qb then theta = 0 and we can return qa
            if (abs(cosHalfTheta) >= 1.0){
                qm.w() = qa.w();qm.x() = qa.x();qm.y() = qa.y();qm.z() = qa.z();
                return qm;
            }
            // Calculate temporary values.
            double halfTheta = acos(cosHalfTheta);
            double sinHalfTheta = sqrt(1.0 - cosHalfTheta*cosHalfTheta);
            // if theta = 180 degrees then result is not fully defined
            // we could rotate around any axis normal to qa or qb
            if (fabs(sinHalfTheta) < 0.001){ // fabs is floating point absolute
                qm.w() = (qa.w() * 0.5 + qb.w() * 0.5);
                qm.x() = (qa.x() * 0.5 + qb.x() * 0.5);
                qm.y() = (qa.y() * 0.5 + qb.y() * 0.5);
                qm.z() = (qa.z() * 0.5 + qb.z() * 0.5);
                return qm;
            }
            double ratioA = sin((1 - t) * halfTheta) / sinHalfTheta;
            double ratioB = sin(t * halfTheta) / sinHalfTheta;
            //calculate Quaternion.
            qm.w() = (qa.w() * ratioA + qb.w() * ratioB);
            qm.x() = (qa.x() * ratioA + qb.x() * ratioB);
            qm.y() = (qa.y() * ratioA + qb.y() * ratioB);
            qm.z() = (qa.z() * ratioA + qb.z() * ratioB);
            return qm;
        }
    }

    static robotPosture  postureInterpolation(robotPosture  &a, robotPosture  &b, double time)
    {
        robotPosture  afterInterp;
        //[0 1] to sigmoid function
        const double SLOPE=3.5;
        double origin= ((  1./(1.+exp(-((time-0.5 )*2.)*SLOPE) ) )-0.5 );
        double maxA= ((  1./(1.+exp(-((1.-0.5 )*2.)*SLOPE) ) )-0.5 );
        double scale=1./maxA;

        double sigmoidTime=((origin)*(scale)+1)/2.;

        Vector3d intePos= linearInterpolation(a.getPosition(),b.getPosition() ,sigmoidTime);
        afterInterp.setPosition ( intePos);
        Quaterniond inteQua=slerp(a.getQuaternion(),b.getQuaternion(),sigmoidTime );

        afterInterp.setQuaternion(inteQua  );
        return afterInterp;
    }

    static Vector4d convert2VectorForDisplay(Quaterniond q)
    {
        Vector4d v;
        v(0)=q.w();v(1)=q.x();v(2)=q.y();v(3)=q.z();
        return v;
    }

    static Vector7d convert2VectorForDisplay(robotPosture  q)
    {
        Vector7d v;
        v(0)=q.getQuaternion().w();
        v(1)=q.getQuaternion().x();
        v(2)=q.getQuaternion().y();
        v(3)=q.getQuaternion().z();
        v(4)=q.getPosition()(0);
        v(5)=q.getPosition()(1);
        v(6)=q.getPosition()(2);
        return v;
    }
    static robotPosture convertHomoMatrix2RobotPosture(Matrix4d &mat)
    {
        Matrix3d matTemp;
        for (int i=0;i<3;i++)
           for (int j=0;j<3;j++)
               matTemp(i,j)=mat(i,j);

        robotPosture result;
        Vector3d vTemp( mat(0,3),  mat(1,3),  mat(2,3) );
        result.setPosition(  vTemp );
        Quaterniond qTemp(matTemp);
        result.setQuaternion( qTemp);
        return result;
    }

    static Matrix4d convertRobotPosture2HomoMatrix( robotPosture & posture)
    {
        Matrix3d rot =posture.getQuaternion().toRotationMatrix();
        Matrix4d homoMat;
        for (int i=0;i<3;i++)
            for (int j=0;j<3;j++)
            {
              homoMat(i,j) =rot(i,j);
            }
        homoMat(0,3)=posture.getPosition()(0);
        homoMat(1,3)=posture.getPosition()(1);
        homoMat(2,3)=posture.getPosition()(2);
        homoMat(3,3)=1;

        homoMat(3,0) = 0;
        homoMat(3,1) = 0;
        homoMat(3,2) = 0;
        return homoMat;
    }


    static cv::Mat readTransformMat(char *fname)
    {
        cv::Mat mat = cv::Mat::zeros(4,4, CV_64F);
        ifstream f_stream(fname);
          for (int i=0; i<4; i++)
              for (int j=0; j<4; j++)
                  {
                      double variable;
                      f_stream >> variable;
                      mat.at<double>(i,j)=variable;
                  }
        return mat;
    }

    static Matrix4d readTransformEigen(char *fname)
    {
        Matrix4d eigen;
        ifstream f_stream(fname);
        for (int i=0; i<4; i++)
        {
            for (int j=0; j<4; j++)
                {
                    f_stream >> eigen(i,j);
                }
        }
        return eigen;
    }

    static Matrix4d rotateTheMatrix(Matrix4d &a, double angle1, double angle2, double angle3)
    {

    }



    static double SafeAcos (double x)
      {
      if (x < -1.0) x = -1.0 ;
      else if (x > 1.0) x = 1.0 ;
      return acos (x) ;
      }


    static cv::Mat Eigen2CVMat(Matrix4d &mat)
    {
        cv::Mat EEPostureCV = cv::Mat(4,4, CV_64F);
        for (int i=0; i<4;i++)
            for (int j=0; j<4;j++ )
            EEPostureCV.at<double> (i,j)= mat(i,j) ;
        return EEPostureCV;
    }

    static cv::Mat Eigen2CVMat(Matrix3d &mat)
    {
        cv::Mat EEPostureCV = cv::Mat(3,3, CV_64F);
        for (int i=0; i<3;i++)
            for (int j=0; j<3;j++ )
            EEPostureCV.at<double> (i,j)= mat(i,j) ;
        return EEPostureCV;
    }

    static Eigen::MatrixXd CVMat2Eigen(cv::Mat &mat)
    {
        //cout << mat << endl;
        int row = mat.rows;
        int col = mat.cols;
        MatrixXd EigenMatrix(row, col);
        for (int i=0; i<mat.rows;i++)
            for (int j=0; j<mat.cols;j++ )
            {
                EigenMatrix(i,j) = mat.at<double>(i,j);
           //     cout << i << " " << j << " " << mat.at<double>(i,j) << endl;
            }
        return EigenMatrix;
    }

    static Erl::Transformd Eigen2Erl(Matrix4d &mat_ei)
    {
//        Erl::Transformd mat_erl;
        Matrix3d rotmat_ei;
        for (int i=0; i<3;i++)
            for (int j=0; j<3;j++ )
            rotmat_ei(i,j)= mat_ei(i,j) ;

        Erl::Transformd mat_erl;
        mat_erl.setRotation(rotmat_ei);
        mat_erl.setX(mat_ei(0,3)*1000);
        mat_erl.setY(mat_ei(1,3)*1000);
        mat_erl.setZ(mat_ei(2,3)*1000);

        return mat_erl;
    }

    static Matrix4d Erl2Eigen(Erl::Transformd &mat_erl)
    {
        Matrix4d mat_ei;
        for (int i=0; i<3;i++)
            for (int j=0; j<3;j++ )
            mat_ei(i,j)= mat_erl.getRotation()(i,j) ;

        mat_ei(0,3) = mat_erl.getTranslation()(0)/1000;
        mat_ei(1,3) = mat_erl.getTranslation()(1)/1000;
        mat_ei(2,3) = mat_erl.getTranslation()(2)/1000;
        mat_ei(3,3) = 1;

        return mat_ei;
    }

    static robotPosture Erl2RobotPosture(Erl::Transformd &mat_erl)
    {
        Matrix4d mat_ei = Erl2Eigen(mat_erl);
        robotPosture robpose = Eigen2RobotPosture(mat_ei);
        return robpose;
    }

    static Erl::Quaterniond EigenQuat2ErlQuat(Eigen::Quaterniond &quat_eigen)
    {
        Erl::Quaterniond quat_erl;
        quat_erl.x() = quat_eigen.x();
        quat_erl.y() = quat_eigen.y();
        quat_erl.z() = quat_eigen.z();
        quat_erl.w() = quat_eigen.w();
        return quat_erl;
    }

    static Eigen::Quaterniond ErlQuat2EigenQuat(Erl::Quaternion<double> &quat_erl)
    {
        Eigen::Quaterniond quat_eigen;
        quat_eigen.x() = quat_erl.x();
        quat_eigen.y() = quat_erl.y();
        quat_eigen.z() = quat_erl.z();
        quat_eigen.w() = quat_erl.w();
        return quat_eigen;
    }

    static cv::Mat RobotPosture2CVMat(robotPosture rob_pose)
    {
        Matrix4d rob_ei = Functions::RobotPosture2Eigen(rob_pose);
        cv::Mat EEPostureCV = cv::Mat(4,4, CV_64F);
        for (int i=0; i<4;i++)
            for (int j=0; j<4;j++ )
            EEPostureCV.at<double> (i,j)= rob_ei(i,j) ;
        return EEPostureCV;
    }

    static Erl::Transformd RobotPosture2ErlTransformd(robotPosture rob_pose)
    {
        Matrix4d rob_ei = Functions::RobotPosture2Eigen(rob_pose);
        Erl::Transformd rob_erl;
        for (int i=0; i<3; i++)
        {
            rob_erl(i,3) = rob_ei(i,3) * 1000;
            for (int j=0; j<3; j++)
                rob_erl(i,j) = rob_ei(i,j);
        }
        return rob_erl;
    }

    static robotPosture CVMat2RobotPosture(cv::Mat &mat)
    {
        Matrix4d EigenMatrix(4,4);
        for (int i=0; i<4;i++)
            for (int j=0; j<4;j++ )
            {
                EigenMatrix(i,j) = mat.at<double>(i,j);
            }
        robotPosture rob_pose = Functions::Eigen2RobotPosture(EigenMatrix);
        return rob_pose;
    }

    static cv::Mat ToolPose2CVMat(aruco::Marker tool)
    {
        cv::Mat cvMat = cv::Mat::eye(4,4, CV_64F);
        cvMat.at<double>(0,3) = (double)tool.Tvec.ptr<float>(0)[0];
        cvMat.at<double>(1,3) = (double)tool.Tvec.ptr<float>(0)[1];
        cvMat.at<double>(2,3) = (double)tool.Tvec.ptr<float>(0)[2];

        cv::Mat cvMat_3x3;
        Rodrigues(tool.Rvec, cvMat_3x3);
        for(int i=0; i<3; i++)
            for (int j=0; j<3; j++)
                cvMat.at<double>(i,j) = (double)cvMat_3x3.at<float>(i,j);

        return cvMat;
    }


    static aruco::Marker CVMat2ToolPose(cv::Mat cvMat)
    {
        aruco::Marker tool;
        tool.Tvec.ptr<float>(0)[0] = cvMat.at<double>(0,3);
        tool.Tvec.ptr<float>(0)[1] = cvMat.at<double>(1,3);
        tool.Tvec.ptr<float>(0)[2] = cvMat.at<double>(2,3);
        Rodrigues(cvMat.colRange(0,3).rowRange(0,3), tool.Rvec);
        return tool;
    }

    static Eigen::Matrix4d ToolPose2Eigen(aruco::Marker tool)
    {
        cv::Mat mat_ = ToolPose2CVMat(tool);
        Matrix4d mat_ei = CVMat2Eigen(mat_);
        return mat_ei;
    }

    static aruco::Marker Eigen2ToolPose(Matrix4d mat_eigen)
    {
        cv::Mat cvMat = Eigen2CVMat(mat_eigen);
        aruco::Marker tool;
        tool.Tvec.ptr<float>(0)[0] = cvMat.at<double>(0,3);
        tool.Tvec.ptr<float>(0)[1] = cvMat.at<double>(1,3);
        tool.Tvec.ptr<float>(0)[2] = cvMat.at<double>(2,3);

        cv::Mat cvMat_3x3 = cv::Mat::zeros(3,3,CV_32F);
        for(int i=0; i<3; i++)
            for (int j=0; j<3; j++)
                cvMat_3x3.at<float>(i,j) = (float)cvMat.at<double>(i,j);

        Rodrigues(cvMat_3x3, tool.Rvec);
        return tool;
    }

    static robotPosture Eigen2RobotPosture(Matrix4d &mat)
    {
        Matrix3d matTemp;
        for (int i=0;i<3;i++)
           for (int j=0;j<3;j++)
               matTemp(i,j)=mat(i,j);

        robotPosture result;
        Vector3d vTemp( mat(0,3),  mat(1,3),  mat(2,3) );
        result.setPosition(  vTemp );
        Quaterniond qTemp(matTemp);
        result.setQuaternion( qTemp);
        return result;
    }

    static Matrix4d RobotPosture2Eigen( robotPosture & posture)
    {
        Matrix3d rot =posture.getQuaternion().toRotationMatrix();
        Matrix4d homoMat;
        for (int i=0;i<3;i++)
            for (int j=0;j<3;j++)
            {
              homoMat(i,j) =rot(i,j);
            }
        homoMat(0,3)=posture.getPosition()(0);
        homoMat(1,3)=posture.getPosition()(1);
        homoMat(2,3)=posture.getPosition()(2);
        homoMat(3,3)=1;

        homoMat(3,0) = 0;
        homoMat(3,1) = 0;
        homoMat(3,2) = 0;
        return homoMat;
    }

    static Matrix4d readPose(char *fname)
    {
      Matrix4d mat;
      ifstream f_stream(fname);
      for (int i=0; i<4; i++)
          for (int j=0; j<4; j++)
              {
                  double variable;
                  f_stream >> variable;
                  mat(i,j)=variable;
              }
      return mat;
    }


    // Calculates rotation matrix given euler angles.
    static cv::Mat eulerAnglesToRotationMatrix(cv::Vec3d &theta)
    {
        // Calculate rotation about x axis
        cv::Mat R_x = (cv::Mat_<double>(3,3) <<
                   1,       0,              0,
                   0,       cos(theta[0]),   -sin(theta[0]),
                   0,       sin(theta[0]),   cos(theta[0])
                   );

        // Calculate rotation about y axis
        cv::Mat R_y = (cv::Mat_<double>(3,3) <<
                   cos(theta[1]),    0,      sin(theta[1]),
                   0,               1,      0,
                   -sin(theta[1]),   0,      cos(theta[1])
                   );

        // Calculate rotation about z axis
        cv::Mat R_z = (cv::Mat_<double>(3,3) <<
                   cos(theta[2]),    -sin(theta[2]),      0,
                   sin(theta[2]),    cos(theta[2]),       0,
                   0,               0,                  1);

        // Combined rotation matrix
        cv::Mat R = R_z * R_y * R_x;

        return R;
    }


    // Calculates rotation matrix to euler angles
    // The result is the same as MATLAB except the order
    // of the euler angles ( x and z are swapped ).
    static cv::Vec3d rotationMatrixToEulerAngles(cv::Mat &R)
    {

        assert(isRotationMatrix(R));

        double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

        bool singular = sy < 1e-6; // If

        double x, y, z;
        if (!singular)
        {
            x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
            y = atan2(-R.at<double>(2,0), sy);
            z = atan2(R.at<double>(1,0), R.at<double>(0,0));
        }
        else
        {
            x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
            y = atan2(-R.at<double>(2,0), sy);
            z = 0;
        }
        return cv::Vec3d(x, y, z);
    }





    // Checks if a matrix is a valid rotation matrix.
    static bool isRotationMatrix(cv::Mat &R)
    {
        cv::Mat Rt;
        transpose(R, Rt);
        cv::Mat shouldBeIdentity = Rt * R;
        cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

        cout << "norm(I, shouldBeIdentity) " << norm(I, shouldBeIdentity) << endl;
        return  norm(I, shouldBeIdentity) < 1e-3;

    }

    static void ToolPose2File(std::ofstream& fstream, aruco::Marker m){

        fstream << m.Tvec.ptr<float>(0)[0] << " "
           << m.Tvec.ptr<float>(0)[1] << " "
           << m.Tvec.ptr<float>(0)[2] << " "
           << m.Rvec.ptr<float>(0)[0] << " "
           << m.Rvec.ptr<float>(0)[1] << " "
           << m.Rvec.ptr<float>(0)[2] << " "<<endl;

    }

};

#endif
