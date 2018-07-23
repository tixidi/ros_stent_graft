#include "kukaKinematicModel.h"
//////////////////////////////////////////////////////////////////////////////////////////////////////
kukaKinematicModel::kukaKinematicModel(int port,const char *kukaIPAddress,const char *localIPAddress, Matrix4d  &coord)
{
    robotReference=port;
    kukaRobot=new friRemote(port,kukaIPAddress,localIPAddress);
    statusVariable = -1;
    init = true;
    running=false;
    gloCoor=coord;
//    Matrix4d mat=forwardKinematics(kukaMsr.jointAngles);
//    Matrix3d rot=MatrixXd::Zero(3,3);
//    for (int i=0;i<3;i++)
//        for (int j=0;j<3;j++)
//            rot(i,j)=mat(i,j);
    preq.w()=1;
    preq.x()=0;
    preq.y()=0;
    preq.z()=0;

    sign=1;
    timeCnt= new QTime();
    timeCnt->start();
    prevTime=0;
//    kukaMsr.jointAngles<<-8.82*3.1415926/180, (167.96-90)*3.1415926/180, -94.33*3.1415926/180,-86.19*3.1415926/180,122.09*3.1415926/180 ,6.81*3.1415926/180, -43.70*3.1415926/180;
//    kukaMsr.jointAngles<<-67*3.1415926/180, (125.96-90)*3.1415926/180, -19.33*3.1415926/180,-80.19*3.1415926/180,119.09*3.1415926/180 ,60*3.1415926/180, 5.70*3.1415926/180;
    kukaMsr.jointAngles<<-108*3.1415926/180, 50*3.1415926/180, 70*3.1415926/180,-71*3.1415926/180, 55*3.1415926/180 ,14*3.1415926/180, -37*3.1415926/180;

    kukaD<< 0.3105,    0,    0.40,   0,   0.39,   0,      1.1785-1.1005 +0.01;
    kukaA<<0,       0,    0,      0,   0,      0,      0;
    kukaAfa<<0,       3.1415926/2, -3.1415926/2, -3.1415926/2,  3.1415926/2, 3.1415926/2,   -3.1415926/2;  //for old Kuka
}

Matrix4d kukaKinematicModel::getGlobalBase()
{
    return gloCoor;
}

Matrix4d kukaKinematicModel::getGlobalTip()
{
    return gloCoor*getHomoMatrix();
}

void kukaKinematicModel::robotStop()
{
    kukaRobot->setToKRLInt(0,-2);
    kukaRobot->doSendData();
    cout << "KUKA"<<robotReference-49938  << " state: "<< "FRI is deactiviated"<<endl;
}

void kukaKinematicModel::robotDisconnect()
{
    kukaRobot->setToKRLInt(0,-2);
    kukaRobot->setToKRLInt(0,3); 
    cout << "KUKA"<<robotReference-49938  << " state: "<< "FRI is closed"<<endl;
}

void kukaKinematicModel::robotConnect()
{
    kukaRobot->setToKRLInt(0,2);
    cout << "KUKA"<<robotReference-49938  << " state: "<< "FRI is activiated"<<endl;
}

bool kukaKinematicModel::robotReady()
{
   if (running==true)
   {
       return true;
   }
   else
       return false;
}
int kukaKinematicModel:: robotState()
{
    return kukaRobot->getState();
}

////private function
void kukaKinematicModel::run()
{
    while (init)
    {
        kukaRobot->doDataExchange(); // Updating variable
        if (kukaRobot->getState()>=FRI_STATE_MON)
        {
            statusVariable = kukaRobot->getFrmKRLInt(0); // Evaluating the Robot Status
        }
        else
        {
            statusVariable = -1;
        }
        if (statusVariable>0)
        {
            init=false;
            running=true;
            readFromKuka();
            kukaCmd.jointAngles=getJointAngles();
            previousJoints=getJointAngles();
        }
        else
        {
            init=true;
        }
    }

    while(running)
    {
        bool a=false;
        #ifdef SimulationON
        a=true;
        #endif
        if (a==false)
        {
            SendToKuka();
            int currTime=timeCnt->elapsed();
            if ( (currTime-prevTime)>5  )
            {
                prevTime=currTime;
                readFromKuka();
            }
        }
       msleep(1);
    }
}

void kukaKinematicModel::readFromKuka()
{  
    #ifdef SimulationON
        kukaMsr.jointAngles=kukaMsr.jointAngles;       
    #else
        float *jointA=kukaRobot->getMsrMsrJntPosition();
        for (int i=0;i<7;i++)
        kukaMsr.jointAngles(i)=jointA[i];
    #endif
}

MatrixXd kukaKinematicModel::getJacobian()
{
    kukaMsr.Jacobian=calcuJacobin(kukaMsr.jointAngles);
    return kukaMsr.Jacobian;
}

Vector7d kukaKinematicModel::getJointAngles()
{
    return kukaMsr.jointAngles;
}

Vector3d kukaKinematicModel::getCartPosition()
{
    Matrix4d mat=forwardKinematics(kukaMsr.jointAngles);
    kukaMsr.cartPosition(0)=mat(0,3);
    kukaMsr.cartPosition(1)=mat(1,3);
    kukaMsr.cartPosition(2)=mat(2,3);
    return kukaMsr.cartPosition;
}

Matrix4d kukaKinematicModel::getHomoMatrix()
{
    Matrix4d mat=forwardKinematics(kukaMsr.jointAngles);
    return mat;
}

Matrix3d kukaKinematicModel::getRotMatrix()
{
    Matrix4d mat=forwardKinematics(kukaMsr.jointAngles);
    for (int i=0;i<3;i++)
    for(int j=0;j<3;j++)
    kukaMsr.rotMatrix(i,j)=mat(i,j);
    return kukaMsr.rotMatrix;
}

Quaterniond kukaKinematicModel::getQuaternion()
{
    Matrix4d mat=forwardKinematics(kukaMsr.jointAngles);
    Matrix3d rot;
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            rot(i,j)=mat(i,j);
    Quaterniond qua(rot);
    //solution 2
    if ( qua.x()* preq.x()+qua.y()* preq.y()+qua.z()* preq.z() <0 )
    {
        qua.x()=-qua.x();
        qua.y()=-qua.y();
        qua.z()=-qua.z();
        qua.w()=-qua.w();
    }
    preq=qua;
    return qua;
}

robotPosture  kukaKinematicModel::getRobtPosture()
{   
    robotPosture  Q;
    Quaterniond qTemp=getQuaternion();
    Vector3d   vTemp= getCartPosition() ;
    Q.setQuaternion( qTemp) ;
    Q.setPosition(vTemp  );
    return Q;
}

void kukaKinematicModel::SendToKuka()
{
   double temp[7];
   for (int i=0;i<7;i++)
        temp[i]=kukaCmd.jointAngles(i);
   //if (robotReference==49938)
   //cout<<kukaCmd.jointAngles.transpose()*180/3.1415926<<endl;
   kukaRobot->doPositionControl(temp);  
}

void kukaKinematicModel::setJointAngles(Vector7d &joint)
{
    //joint range hard limit
    VectorXd tempJ=VectorXd::Zero(7);
    tempJ=joint;
///// joint range limits constraints//////////////////////////////////
    double safeT=1;
    Vector7d jointRange;

    jointRange<<(170-safeT)*3.1415926/180, (120-safeT)*3.1415926/180,(170-safeT)*3.1415926/180,(120-safeT)*3.1415926/180,(170-safeT)*3.1415926/180,(120-safeT)*3.1415926/180,(170-safeT)*3.1415926/180;
    for (int i=0;i<7;i++)
    {
        if ( abs (tempJ(i) )> jointRange(i) )
           {
            tempJ(i)= jointRange(i)*tempJ(i)/abs(tempJ(i));          
            //cout<<"Robot " <<robotreference <<": " <<joint.transpose()*180/PI <<endl;
           }
    }

    //cout<<"Robot " <<robotreference <<"   Joint "<<i<<" velocity limit voilates " <<jointVlim<<endl;
/////  joint velocity voilation protection ///////////////////////////////////
    //set kuka
//    if (disAble==false)
//    {

//        if (robotReference==49938)
//            cout<<robotReference  <<"  "<< kukaCmd.jointAngles.transpose()<<endl;

//        if (robotReference==49939)
//            cout<<robotReference  <<"  "<<  kukaCmd.jointAngles.transpose()<<endl;

        #ifdef SimulationON
               kukaMsr.jointAngles=tempJ;
        #else
               kukaCmd.jointAngles=tempJ;
        #endif
        previousJoints=tempJ;
//    }
//    else
//    {   //when need to stop, current joint position is used
//        readFromKuka();
//        kukaCmd.jointAngles=getJointAngles();
//    }
}

Matrix67d kukaKinematicModel::calcuJacobin(Vector7d &jointAngle)
{
    Vector3d z0(0., 0. ,1.);
    //Vector3d z1=rotMatrix(joint(0), 0 ) *z0;
    Vector3d z1=rotMatrix(jointAngle(0), 0 ) *rotMatrix(jointAngle(1), 1 )*z0;
    Vector3d z2=rotMatrix(jointAngle(0), 0 ) *rotMatrix(jointAngle(1), 1 )*  rotMatrix(jointAngle(2), 2 ) *z0;
    Vector3d z3=rotMatrix(jointAngle(0), 0 ) *rotMatrix(jointAngle(1), 1 )*  rotMatrix(jointAngle(2), 2 ) *rotMatrix(jointAngle(3), 3 )  *z0;
    Vector3d z4=rotMatrix(jointAngle(0), 0 ) *rotMatrix(jointAngle(1), 1 )*  rotMatrix(jointAngle(2), 2 ) *rotMatrix(jointAngle(3), 3 )  *rotMatrix(jointAngle(4), 4 )  *z0;
    Vector3d z5=rotMatrix(jointAngle(0), 0 ) *rotMatrix(jointAngle(1), 1 )*  rotMatrix(jointAngle(2), 2 ) *rotMatrix(jointAngle(3), 3 )  *rotMatrix(jointAngle(4), 4 )*rotMatrix(jointAngle(5), 5 ) *z0;
    Vector3d z6=rotMatrix(jointAngle(0), 0 ) *rotMatrix(jointAngle(1), 1 )*  rotMatrix(jointAngle(2), 2 ) *rotMatrix(jointAngle(3), 3 )  *rotMatrix(jointAngle(4), 4 )*rotMatrix(jointAngle(5), 5 ) *rotMatrix(jointAngle(6), 6 ) *z0;
    Vector3d JO[7];
    JO[0]=z0;
    JO[1]=z1;
    JO[2]=z2;
    JO[3]=z3;
    JO[4]=z4;
    JO[5]=z5;
    JO[6]=z6;

    Vector4d p[7];
    p[0](0)=0.;  p[0](1)=0.;  p[0](2)=0.;  p[0](3)=1.;

    Vector4d pe=hoMatrix(jointAngle(0),0 )*hoMatrix(jointAngle(1),1 )*hoMatrix(jointAngle(2),2 ) *hoMatrix(jointAngle(3),3 )*hoMatrix(jointAngle(4),4 ) *hoMatrix(jointAngle(5),5 )*hoMatrix(jointAngle(6),6 )* p[0];
     p[1]=hoMatrix(jointAngle(0),0 )* p[0];
     p[2]=hoMatrix(jointAngle(0),0 )*hoMatrix(jointAngle(1),1 )* p[0];
     p[3]=hoMatrix(jointAngle(0),0 )*hoMatrix(jointAngle(1),1 )*hoMatrix(jointAngle(2),2 )* p[0];
     p[4]=hoMatrix(jointAngle(0),0 )*hoMatrix(jointAngle(1),1 )*hoMatrix(jointAngle(2),2 )*hoMatrix(jointAngle(3),3 )* p[0];
     p[5]=hoMatrix(jointAngle(0),0 )*hoMatrix(jointAngle(1),1 )*hoMatrix(jointAngle(2),2 )*hoMatrix(jointAngle(3),3 )*hoMatrix(jointAngle(4),4 )* p[0];
     p[6]=hoMatrix(jointAngle(0),0 )*hoMatrix(jointAngle(1),1 )*hoMatrix(jointAngle(2),2 )*hoMatrix(jointAngle(3),3 )*hoMatrix(jointAngle(4),4 )*hoMatrix(jointAngle(5),5 )* p[0];

    Vector3d tempVec[7];
    for (int i=0;i<=6;i++)
    {
        tempVec[i](0)=pe(0)-p[i](0);
        tempVec[i](1)=pe(1)-p[i](1);
        tempVec[i](2)=pe(2)-p[i](2);
    }

    Vector3d JP[7];
    JP[0]=z0.cross(tempVec[0] );
    JP[1]=z1.cross(tempVec[1] );
    JP[2]=z2.cross(tempVec[2] );
    JP[3]=z3.cross(tempVec[3] );
    JP[4]=z4.cross(tempVec[4] );
    JP[5]=z5.cross(tempVec[5] );
    JP[6]=z6.cross(tempVec[6] );


    Matrix67d Jacobin;
    for (int i=0;i<7;i++)
    {
        Jacobin(0, i)=JP[i](0);
        Jacobin(1, i)=JP[i](1);
        Jacobin(2, i)=JP[i](2);
        Jacobin(3, i)=JO[i](0);
        Jacobin(4, i)=JO[i](1);
        Jacobin(5, i)=JO[i](2);
    }
    return Jacobin;
}

Matrix3d kukaKinematicModel::rotMatrix(double jointAngle, int i)
{
    Matrix4d homoG=Rx (kukaAfa[i])*Dx(kukaA[i])*Rz(jointAngle)*Qz(kukaD[i]);
    Matrix3d rot;
    rot(0,0)=homoG(0,0);rot(0,1)=homoG(0,1);rot(0,2)=homoG(0,2);
    rot(1,0)=homoG(1,0);rot(1,1)=homoG(1,1);rot(1,2)=homoG(1,2);
    rot(2,0)=homoG(2,0);rot(2,1)=homoG(2,1);rot(2,2)=homoG(2,2);
    return rot;
}

Matrix4d kukaKinematicModel::hoMatrix (double jointAngle, int i)
{
    Matrix4d homo=Rx (kukaAfa[i])*Dx(kukaA[i])*Rz(jointAngle)*Qz(kukaD[i]);
    return homo;
}


Matrix4d  kukaKinematicModel::forwardKinematics(Vector7d &joint)
{
    Matrix4d homoMatrix;
    Matrix4d T01=Rx (kukaAfa(0))*Dx(kukaA(0))*Rz(joint(0))*Qz(kukaD(0));
    Matrix4d T12=Rx (kukaAfa(1))*Dx(kukaA(1))*Rz(joint(1))*Qz(kukaD(1));
    Matrix4d T23=Rx (kukaAfa(2))*Dx(kukaA(2))*Rz(joint(2))*Qz(kukaD(2));
    Matrix4d T34=Rx (kukaAfa(3))*Dx(kukaA(3))*Rz(joint(3))*Qz(kukaD(3));
    Matrix4d T45=Rx (kukaAfa(4))*Dx(kukaA(4))*Rz(joint(4))*Qz(kukaD(4));
    Matrix4d T56=Rx (kukaAfa(5))*Dx(kukaA(5))*Rz(joint(5))*Qz(kukaD(5));
    Matrix4d T67=Rx (kukaAfa(6))*Dx(kukaA(6))*Rz(joint(6))*Qz(kukaD(6));
    return homoMatrix=T01*T12*T23*T34*T45*T56*T67;
}

Matrix4d  kukaKinematicModel::Rx(double tempAfa)
{
    Matrix4d tempMatrixRx;
    tempMatrixRx (0,0)=1;    tempMatrixRx (0,1)=0;              tempMatrixRx (0,2)=0;                 tempMatrixRx (0,3)=0;
    tempMatrixRx (1,0)=0;    tempMatrixRx (1,1)=cos(tempAfa);   tempMatrixRx (1,2)=-sin(tempAfa);     tempMatrixRx (1,3)=0;
    tempMatrixRx (2,0)=0;    tempMatrixRx (2,1)=sin(tempAfa);   tempMatrixRx (2,2)=cos(tempAfa);      tempMatrixRx (2,3)=0;
    tempMatrixRx (3,0)=0;    tempMatrixRx (3,1)=0;              tempMatrixRx (3,2)=0;                 tempMatrixRx (3,3)=1;
    return tempMatrixRx;
}

Matrix4d  kukaKinematicModel::Dx(double tempA)
{
    Matrix4d tempMatrixDx;
    tempMatrixDx (0,0)=1;    tempMatrixDx (0,1)=0;          tempMatrixDx (0,2)=0;             tempMatrixDx (0,3)=tempA;
    tempMatrixDx (1,0)=0;    tempMatrixDx (1,1)=1;          tempMatrixDx (1,2)=0;             tempMatrixDx (1,3)=0;
    tempMatrixDx (2,0)=0;    tempMatrixDx (2,1)=0;          tempMatrixDx (2,2)=1;             tempMatrixDx (2,3)=0;
    tempMatrixDx (3,0)=0;    tempMatrixDx (3,1)=0;          tempMatrixDx (3,2)=0;             tempMatrixDx (3,3)=1;
    return tempMatrixDx;
}

Matrix4d  kukaKinematicModel::Rz(double tempTheta)
{
    Matrix4d tempMatrixRz;
    tempMatrixRz (0,0)=cos(tempTheta);    tempMatrixRz (0,1)=-sin(tempTheta);          tempMatrixRz (0,2)=0;             tempMatrixRz (0,3)=0;
    tempMatrixRz (1,0)=sin(tempTheta);    tempMatrixRz (1,1)= cos(tempTheta);          tempMatrixRz (1,2)=0;             tempMatrixRz (1,3)=0;
    tempMatrixRz (2,0)=0;                 tempMatrixRz (2,1)=0;                        tempMatrixRz (2,2)=1;             tempMatrixRz (2,3)=0;
    tempMatrixRz (3,0)=0;                 tempMatrixRz (3,1)=0;                        tempMatrixRz (3,2)=0;             tempMatrixRz (3,3)=1;
    return tempMatrixRz;
}

Matrix4d  kukaKinematicModel::Qz(double tempD)
{
    Matrix4d tempMatrixQz;
    tempMatrixQz (0,0)=1;    tempMatrixQz (0,1)=0;          tempMatrixQz (0,2)=0;             tempMatrixQz (0,3)=0;
    tempMatrixQz (1,0)=0;    tempMatrixQz (1,1)=1;          tempMatrixQz (1,2)=0;             tempMatrixQz (1,3)=0;
    tempMatrixQz (2,0)=0;    tempMatrixQz (2,1)=0;          tempMatrixQz (2,2)=1;             tempMatrixQz (2,3)=tempD;
    tempMatrixQz (3,0)=0;    tempMatrixQz (3,1)=0;          tempMatrixQz (3,2)=0;             tempMatrixQz (3,3)=1;
    return tempMatrixQz;
}


