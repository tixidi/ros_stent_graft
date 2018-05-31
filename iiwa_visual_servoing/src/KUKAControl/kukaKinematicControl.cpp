#include "kukaKinematicControl.h"

KukaKinematicControl::KukaKinematicControl(int port,const char *kukaIPAddress,const char *localIPAddress, Matrix4d  &coord)
{   
    kukaRobot= new kukaKinematicModel(port,kukaIPAddress,localIPAddress,coord );
    kukaRobot->start(QThread::TimeCriticalPriority);
    robotReference=port-49938;
    weight=MatrixXd::Zero(7,7);
    weight(0,0)=20.;
    weight(1,1)=30.;
    weight(2,2)=10;
    weight(3,3)=10.;
    weight(4,4)=7.;
    weight(5,5)=100.;
    weight(6,6)=1.;
//    weight(0,0)=1.;
//    weight(1,1)=1.;
//    weight(2,2)=1;
//    weight(3,3)=1.;
//    weight(4,4)=1.;
//    weight(5,5)=1.;
//    weight(6,6)=1.;

    timeCnt= new QTime();
    timeCnt->start();
    prevTime=timeCnt->elapsed();
    firstTime=false;

    //for move to function
    reachTarget=true;
    reachTargetTip=true;
    stepCnt=0;

    preq.w()=1;
    preq.x()=0;
    preq.y()=0;
    preq.z()=0;

    f_inter.open("../source/demonstrantion/interpolate.txt");
}

void KukaKinematicControl::run()
{
    //wait for conection   
    #ifdef SimulationON
    #else
        Ready=false;
        while (! ( kukaRobot->robotReady() ) )
        {
           msleep (SAMPLETIME);
        }
        msleep (SAMPLETIME*1000);
        cout << "KUKA"<<robotReference << " : "<< "connection is made, FRI can be activiated"<<endl;
    #endif

    //parameter intialization alwasy here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    desPosQuaKuka=kukaRobot->getRobtPosture();
    Ready=true;
    msleep (SAMPLETIME);
    //cout<<"\n Robot: "<< robotReference<< " ready!"<<endl;
    //control begin
    while (true)
    {
        operaSpaceControl();
        msleep (SAMPLETIME);
    }
}

bool KukaKinematicControl::ArmReady()
{
    if (robotReference==0)
    {
        return true;
    }
    return Ready;
}

void KukaKinematicControl::setJointAngles(Vector7d &Joints)
{
    kukaRobot->setJointAngles(Joints);
}

void KukaKinematicControl::setDesTipPosture(robotPosture &PosQua)
{   
#ifdef SimulationON
    //float VELOCITY_TRANS = 0.001; // meter
    float VELOCITY_ROT = 0.1; // radian
    float VELOCITY_CHECK = 0.05;
#else
    //float VELOCITY_TRANS = 0.0001; // meter
    float VELOCITY_ROT = 0.7; // radian
    float VELOCITY_CHECK = 0.02; //0.008
#endif

    iniMovPosture=kukaRobot->getRobtPosture();

//    cout << iniMovPosture.getPosition().transpose()
//         << " " << iniMovPosture.getQuaternion().w()
//         << " " << iniMovPosture.getQuaternion().x()
//         << " " << iniMovPosture.getQuaternion().y()
//         << " " << iniMovPosture.getQuaternion().z()
//         << " " << endl;


    if (PosQua.getQuaternion().dot(
        iniMovPosture.getQuaternion()) < 0)
    {
        PosQua.getQuaternion().w() *= -1;
        PosQua.getQuaternion().x() *= -1;
        PosQua.getQuaternion().y() *= -1;
        PosQua.getQuaternion().z() *= -1;
        cout<<"desPosQuaKuka reverse: " << " "
            << PosQua.getQuaternion().w() << " "
            << PosQua.getQuaternion().x() << " "
            << PosQua.getQuaternion().y() << " "
            << PosQua.getQuaternion().z() << " "
            <<endl;
    }

    // Check velocity ----------------------------
    Matrix4d deltaT = Functions::convertRobotPosture2HomoMatrix(iniMovPosture).inverse() *
                        Functions::convertRobotPosture2HomoMatrix(PosQua);
    robotPosture deltaPose = Functions::convertHomoMatrix2RobotPosture(deltaT);

    if (abs(deltaT(0,3)) > VELOCITY_CHECK || abs(deltaT(1,3)) > VELOCITY_CHECK||abs(deltaT(2,3)) > VELOCITY_CHECK
        || abs(deltaPose.getQuaternion().w()) < VELOCITY_ROT)
    {
        cout << "setDesTipPosture: Too large velocity. " << deltaT(0,3) << " " << deltaT(1,3) <<
                " " << deltaT(2,3) << " " << deltaPose.getQuaternion().w() <<
                " " << deltaPose.getQuaternion().x() <<
                " " << deltaPose.getQuaternion().y() <<
                " " << deltaPose.getQuaternion().z() <<
                " Stop!" << endl;

        cout << "Current pose " << iniMovPosture.getPosition().transpose() <<
                " " << iniMovPosture.getQuaternion().w() <<
                " " << iniMovPosture.getQuaternion().x() <<
                " " << iniMovPosture.getQuaternion().y() <<
                " " << iniMovPosture.getQuaternion().z() << endl;
        cout << "Desired pose " << PosQua.getPosition().transpose() << " "
             << PosQua.getQuaternion().w() << " "
             << PosQua.getQuaternion().x() << " "
             << PosQua.getQuaternion().y() << " "
             << PosQua.getQuaternion().z() << " "
             << endl;

        deltaT(0,3) = deltaT(0,3);
    }
    else
    {
        desPosQuaKuka.setQuaternion(PosQua.getQuaternion());
        desPosQuaKuka.setPosition(PosQua.getPosition());
    }

/*    //fix quaternion ambiguty
    if ( PosQua.getQuaternion().x()* preq.x()+PosQua.getQuaternion().y()* preq.y()+PosQua.getQuaternion().z()* preq.z() <0 )
    {
        desPosQuaKuka.setPosition(PosQua.getPosition()   );
        Quaterniond tempQ=PosQua.getQuaternion();
        tempQ.w()=-tempQ.w();
        tempQ.x()=-tempQ.x();
        tempQ.y()=-tempQ.y();
        tempQ.z()=-tempQ.z();

        desPosQuaKuka.setQuaternion(   tempQ     );
        desPosQuaKuka.setPosition( PosQua.getPosition()    );
        cout<<"desPosQuaKuka reverse: " << PosQua.getPosition().transpose() <<endl;

        preq=tempQ;
        //cout<<Functions::convert2VectorForDisplay(  tempQ).transpose()<<endl;
    }
    else
    {
       //desPosQuaKuka=PosQua;
       desPosQuaKuka.setQuaternion(PosQua.getQuaternion());
       desPosQuaKuka.setPosition(PosQua.getPosition());
       //cout<<"desPosQuaKuka: " << PosQua.getPosition().transpose() <<endl;

       preq=PosQua.getQuaternion();
       //cout<<Functions::convert2VectorForDisplay(  PosQua.getQuaternion()).transpose()<<endl;
    }
  //not fix quaternion ambiguty
     //desPosQuaKuka=PosQua;
*/
}

bool KukaKinematicControl::moveToPosture(robotPosture  &desPosQua)
{
    const double VELOCITY= 0.02;  //0.02 0.008
    const double ANGLEVELOCITY= 6.0; //6 degree/s
    if ( reachTarget==true)
    {
       reachTarget=false;
       iniMovPosture=kukaRobot->getRobtPosture();
       Vector3d cartDistanceVec= (desPosQua.getPosition()-iniMovPosture.getPosition());
       double cartDistance=cartDistanceVec.norm();

       Quaterniond a, b;
       a=desPosQua.getQuaternion();b=iniMovPosture.getQuaternion();
       double product=a.dot(b);

       double theta= Functions::SafeAcos(2.0*product* product  -1.0 )*180./3.1415926 ;

       if (cartDistance<0.01)
       {
           cartDistance=0.02; //0.1
       }
       double time=cartDistance/VELOCITY;//for velocity 20mm/s,
       double time1=theta/ANGLEVELOCITY;

       cout << "cartDistance " << cartDistance << endl;
       cout<<"angle: "<<theta<< "  product: "<< product <<endl;
       cout<<"time car: "<<time<<"  ang: "<<time1<<endl;

#ifdef SimulationON
       if (time>time1)//for rotation and translate, adopt bigger time.
       {
          interpoInterval=time*100./SAMPLETIME;//sampling frequency (0.002s)
       }
       else
       {
          interpoInterval=time1*100./SAMPLETIME;//sampling frequency (0.002s)
       }
# else
       if (time>time1)//for rotation and translate, adopt bigger time.
       {
          interpoInterval=time*200./SAMPLETIME;//sampling frequency (0.002s)
       }
       else
       {
          interpoInterval=time1*200./SAMPLETIME;//sampling frequency (0.002s)
       }
#endif

       if (desPosQua.getQuaternion().dot(
           iniMovPosture.getQuaternion()) < 0)
       {
           desPosQua.getQuaternion().w() *= -1;
           desPosQua.getQuaternion().x() *= -1;
           desPosQua.getQuaternion().y() *= -1;
           desPosQua.getQuaternion().z() *= -1;
       }

//       f_inter << "-1 "
//                << " " << iniMovPosture.getPosition().transpose()
//                << " " << iniMovPosture.getQuaternion().w()
//                << " " << iniMovPosture.getQuaternion().x()
//                << " " << iniMovPosture.getQuaternion().y()
//                << " " << iniMovPosture.getQuaternion().z()
//                << endl;

//       f_inter << "-2 "
//                << " " << desPosQua.getPosition().transpose()
//                << " " << desPosQua.getQuaternion().w()
//                << " " << desPosQua.getQuaternion().x()
//                << " " << desPosQua.getQuaternion().y()
//                << " " << desPosQua.getQuaternion().z()
//                << endl;
    }

    if (reachTarget==false)
    {
        robotPosture  comdPosture;
        comdPosture=Functions::postureInterpolation(iniMovPosture,  desPosQua, 1.0* stepCnt++/interpoInterval );
        if (desPosQua.getQuaternion().dot(
            comdPosture.getQuaternion()) < 0)
        {
            comdPosture.getQuaternion().w() *= -1;
            comdPosture.getQuaternion().x() *= -1;
            comdPosture.getQuaternion().y() *= -1;
            comdPosture.getQuaternion().z() *= -1;
        }

        cout << "moveToPosture " << interpoInterval << " " << stepCnt
             << " " <<comdPosture.getPosition().transpose()
             << " " << comdPosture.getQuaternion().w()
             << " " << comdPosture.getQuaternion().x()
             << " " << comdPosture.getQuaternion().y()
             << " " << comdPosture.getQuaternion().z()
             << endl;

        setDesTipPosture(comdPosture);
//        f_inter << stepCnt
//                << " " <<comdPosture.getPosition().transpose()
//                 << " " << comdPosture.getQuaternion().w()
//                 << " " << comdPosture.getQuaternion().x()
//                 << " " << comdPosture.getQuaternion().y()
//                 << " " << comdPosture.getQuaternion().z()
//                 << endl;

        if (stepCnt>=interpoInterval)
        {
           stepCnt=0;
           reachTarget=true;
        }
    }
    return reachTarget;
}

bool KukaKinematicControl::moveToPostureInTipFrame(robotPosture  &desPosQua)
{
    if (reachTargetTip==true)
    {
        //first translate in global frame
        reachTargetTip=false;
        currTipPosture=getCurrTipPosture();
        Matrix3d mat(currTipPosture.getQuaternion() );
        Vector3d vTemp= mat*desPosQua.getPosition() +currTipPosture.getPosition();      
        currTipPosture.setPosition( vTemp );

        //second rotate in global frame
        Quaterniond qTemp=currTipPosture.getQuaternion()*desPosQua.getQuaternion() ;
        currTipPosture.setQuaternion(qTemp );
    }
    else
    {
       bool result=moveToPosture(currTipPosture);
       if (result==true)
       {
          reachTargetTip=true;
       }
    }
    return reachTargetTip;
}



robotPosture  KukaKinematicControl::getCurrTipPosture()
{
    robotPosture  currPosture=kukaRobot->getRobtPosture();
    return currPosture;
}

Vector7d KukaKinematicControl::getJointAngles()
{
    return kukaRobot->getJointAngles();
}

kukaKinematicModel* KukaKinematicControl::getRobotInstance()
{
    return kukaRobot;
}

void KukaKinematicControl::operaSpaceControl()
{        
    if (firstTime==false)
    {
        firstTime=true;
        prevTime=timeCnt->elapsed();
        desJointPos=kukaRobot->getJointAngles();
    }
    Vector7d desJointRate= calInverseKinematic();
    int time=timeCnt->elapsed();
    int interval=time-prevTime;
    prevTime=time;
    desJointPos= desJointPos+(desJointRate*interval/1000.);
    kukaRobot->setJointAngles( desJointPos );
}

Vector7d KukaKinematicControl::calInverseKinematic( )
{
    MatrixXd J0=kukaRobot->getJacobian();
    MatrixXd J0Inv = Functions:: Wpinv (J0,weight);

    VectorXd r0=VectorXd::Zero(6);
    double gain0=2.,gain01=3.;

    Vector3d Ep0=desPosQuaKuka.getPosition()-kukaRobot->getCartPosition();

    r0(0)=gain0*Ep0(0);
    r0(1)=gain0*Ep0(1);
    r0(2)=gain0*Ep0(2);

    Quaterniond currQ=kukaRobot->getQuaternion();
    Vector3d Eo0= Functions::calQuaternionErr( desPosQuaKuka.getQuaternion(), currQ);

//    if (robotReference==49938)
//    {
//    cout<<"va"<<Functions::convert2VectorForDisplay(desPosQuaKuka.getQuaternion() ).transpose()  <<endl; ;
//    cout<<"vb"<<Functions::convert2VectorForDisplay(currQ ).transpose()   <<endl;
//    }

    r0(3)=gain01*Eo0(0);
    r0(4)=gain01*Eo0(1);
    r0(5)=gain01*Eo0(2);

    VectorXd  desJointRate= J0Inv*r0;

    return desJointRate;
}

void KukaKinematicControl::JointSpaceControl ()
{
    cout<<kukaRobot->getJointAngles().transpose()<<endl;
}

void KukaKinematicControl::robotStop()
{
    kukaRobot->robotStop();
}
void KukaKinematicControl::robotConnect()
{  
    kukaRobot->robotConnect();
}
void KukaKinematicControl::robotDisconnect()
{
    kukaRobot->robotDisconnect();
}

