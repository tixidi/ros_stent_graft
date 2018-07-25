#include "Faulharbermotor.h"

Faulharbermotor::Faulharbermotor()
{
    //ini parameters
    motorConnect=1;
    status=0;
    for (int i=0;i<NODENO;i++)
    {
        cmdPos[i]=0;
        msrPos[i]=0;
        msrTem[i]=0;
        msrCur[i]=0;
    }
    speedLimit[0]=15;
    speedLimit[1]=1000;
    needleDir=1;
    speedScale=0.4;
    prevDataExchangeTime=0;
    //set time tick
    sysTime= new QTime();
    sysTime->start();

    //open port
    //sertialPort1 =new QSerialPort("Com3");
    sertialPort1 =new QSerialPort("/dev/ttyUSB0");

    sertialPort1->open(QIODevice::ReadWrite);

    sertialPort1->setBaudRate(QSerialPort::Baud9600);


    // check connection ready or not by inquiring position
    for (int node=0; node<NODENO; node++)
    {
        int tem0=getPosition(node);
        if (tem0!=ERROR_INDEX && tem0!=TIMEOUT_INDEX )
        {
            cout<<"motor"<< node <<" is OK"<<endl;
            motorConnect=motorConnect*1;
        }
        else
        {
            cout<<"motor"<< node <<" is error"<<endl;
            motorConnect=motorConnect*0;
            exit(1);
        }
    }

    // if all motors are connected, setup device
    if ( motorConnect>0 )
    {
        for (int node=0; node<NODENO; node++)
        {
            //disable feedback information and enable positon reach notify
            char ComPos[15];
            sprintf(ComPos, "%dANSW1\n",node);
            sertialPort1->write(ComPos);
            msleep(100);

            //set  motor home position
//            char ComPos2[15];
//            sprintf(ComPos2, "%dHO\n",node);
//            sertialPort1->write(ComPos2);
//            msleep(100);
        }

        setSpeed(0.88);
        // enable motor
        //disable();
        enable();

        //setCurrentLimit(0, 150);
        msleep(10);
        //start control tick
        controlTick=new QTimer();
        connect(controlTick, SIGNAL(timeout()) , this, SLOT( controlLoop()) );
        controlTick->start(50);


//        QTimer *demo= new QTimer();
//        connect(demo, SIGNAL(timeout()) , this, SLOT( demoLoop()) );
//        demo->start(550);
    }


}

//void Faulharbermotor:: demoLoop()
//{
//    if (isDeviceBusy()==false)
//    {
//        runSingleStitch();

//    }
//}


// set contineous current limit
void Faulharbermotor::setCurrentLimit(int node, int current)
{
    char ComPos[15];
    sprintf(ComPos, "%dLCC%d\n",node,current);
    sertialPort1->write(ComPos);
}

Faulharbermotor::~Faulharbermotor()
{
    disable();
}

void    Faulharbermotor::setSpeed(double scale)
{
    speedScale=scale;
    for (int node=0; node<NODENO; node++)
    {
        double desSpeed= double( speedLimit[node])* scale;
        int desSpeed_Int=int (desSpeed );
        char ComPos1[20];
        sprintf(ComPos1, "%dSP%d\n", node,desSpeed_Int);
        sertialPort1->write(ComPos1);
        msleep(100);
    }
}

void    Faulharbermotor::setHome(int node)
{
    char ComPos2[15];
    sprintf(ComPos2, "%dHO\n",node);
    sertialPort1->write(ComPos2);
    cout<<"homing motor"<<node<<endl;
}

deviceInfomation Faulharbermotor::getAllCtrlInfomation()
{
    deviceInfomation ctrlInfo;
    for (int i=0;i<NODENO;i++)
    {
        ctrlInfo.msrPos[i]=msrPos[i];
        ctrlInfo.cmdPos[i]=cmdPos[i];
        ctrlInfo.msrTem[i]=msrTem[i];
        ctrlInfo.msrCur[i]=msrCur[i];
    }
        ctrlInfo.connectSig=motorConnect;
        ctrlInfo.enableSig=enableSig;
        ctrlInfo.speedScale=speedScale;
        ctrlInfo.status=status;

        ctrlInfo.speedScale=  speedScale;
        ctrlInfo.absLockUpPos=absLockPos;
        ctrlInfo.absLockDownPos=0;
        ctrlInfo.absOpenPos=absOpenPos;
        ctrlInfo.absClosePos=absClosePos;
        return ctrlInfo;
}

bool    Faulharbermotor::isConnected()
{
    if (motorConnect==1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void    Faulharbermotor::enable()
{
    enableSig=true;
    sertialPort1->write("EN\n");
    cout<<"enable all motor"<<endl;
}

void    Faulharbermotor::disable()
{
    enableSig=false;
    sertialPort1->write("DI\n");
    cout<<"disable all motor"<<endl;
}

void    Faulharbermotor::enableToggle()
{
    if (enableSig==true)
    {
        disable();
    }
    else
    {
        enable();
    }
}

void    Faulharbermotor::runSingleStitch()
{
    if (!isDeviceBusy())
    {
        status=1;
				int counter = 0;
        cout<<"status1: "<<"run stitch"<<endl;
				while(controlLoop()|| counter == 10){
						counter++;				
				}
    }

    //direct trigger
//    status=1;
//    controlLoop();

}

double calc_ik(double deg) {
    double deg2rad = M_PI/180;
    double theta0 = 37.46 + deg;
    double L1 = 5.3514, L2 = 12.5, L3 = 2.23285;
    double ang_ECD = abs(theta0 - 90) * deg2rad;
    double L4 = sqrt(L1*L1 + L3*L3 - 2*L1*L3*cos(ang_ECD));
    double ang_CED = M_PI - asin((L1*sin(ang_ECD))/L4);
    double ang_EAD, ang_AED, ang_ADE;

    if (theta0 > 90) {
        ang_AED = 270*deg2rad - ang_CED;
    } else {
        ang_AED = ang_CED - 90*deg2rad;
    }

    ang_EAD = asin((L4*sin(ang_AED))/L2);
    ang_ADE = 180*deg2rad - ang_EAD - ang_AED;

    return sqrt(L2*L2 + L4*L4 - 2*L2*L4*cos(ang_ADE));
}

void    Faulharbermotor::runPierceDeg(double deg)
{

    double L6_0 = calc_ik(0);
    double L6_1 = calc_ik(5);


    double encoder2mm = 121;
    double dL = abs(L6_0 - L6_1) * encoder2mm;

    setSpeed(0.1);
    msleep(10);
    setPosition( absOpenPos-dL, Motor0);
    setSpeed(0.99);
    msleep(10);
}

bool    Faulharbermotor::isDeviceBusy()
{
    if (status==0)
    {
        // device is free
        return false;
    }
    else
    {
        // device is busy
        return true;
    }
}

void    Faulharbermotor::setPosition(int pos, int node)
{
    char ComPos[20];
    sprintf(ComPos, "%dLA%d\n%dM\n", node, pos, node);
    sertialPort1->write(ComPos);
}

void    Faulharbermotor::setVelocity(int vel, int node)
{
    char ComPos[20];
    sprintf(ComPos, "%dV%d\n", node, vel);
    sertialPort1->write(ComPos);
}

int     Faulharbermotor::getTemprature(int node)
{
    int startTime=sysTime->elapsed();
    int curr_temp=0;
    inquireTemprature(node);
    msleep(15);
    while (sertialPort1->waitForReadyRead(TIMEOUT))
    {
        QByteArray temp=sertialPort1->readAll();
        receivedData.append(temp);
        for (int i=1;i<receivedData.size(); i++ )
        {
            if ( (int(receivedData.at(i-1))==13 )&& (int(receivedData.at(i))==10)  )
            {
                //curr_temp=atoi( receivedData.toStdString().c_str() );
               //curr_temp = receivedData.toInt();
                QString DataAsString = QTextCodec::codecForMib(106)->toUnicode(receivedData);
                curr_temp=atof(DataAsString.toStdString().c_str());
                receivedData.clear();

                //int endTime=sysTime->elapsed();
                //int timeElapse= endTime - startTime;
                //cout<< "Motor"<< node<<" temprature:" << curr_temp<< "   Time elapsed:"<<timeElapse<< "ms"<<endl;
                return curr_temp;
            }
        }
        msleep(3);
    }
    cout<< "get temprature timeout: " << node <<endl;
    return TIMEOUT_INDEX;
}

int     Faulharbermotor::getPosition(int node)
{
    int startTime=sysTime->elapsed();
    int curr_pos=0;
    inquirePos(node);
    msleep(15);
    while (sertialPort1->waitForReadyRead(TIMEOUT))
    {
        QByteArray temp=sertialPort1->readAll();
        receivedData.append(temp);
        for (int i=1;i<receivedData.size(); i++ )
        {
            if ( (int(receivedData.at(i-1))==13 )&& (int(receivedData.at(i))==10)  )
            {

                //curr_pos=atoi( receivedData.toStdString().c_str() );
                //curr_pos = receivedData.toInt();

                QString DataAsString = QTextCodec::codecForMib(106)->toUnicode(receivedData);
                curr_pos=atof(DataAsString.toStdString().c_str());
                //cout<< DataAsString.toStdString()<<endl;
                receivedData.clear();

                //int endTime=sysTime->elapsed();
                //int timeElapse= endTime - startTime;
                //cout<< "Motor"<< node<<" position:" << curr_temp<< "      Time elapsed:"<<timeElapse<< "ms"<<endl;
                return curr_pos;
            }
        }
        msleep(3);
    }
    cout<< "get position timeout: " << node <<endl;
    return TIMEOUT_INDEX;
}

int     Faulharbermotor::getCurrent(int node)
{
    int startTime=sysTime->elapsed();
    double curr_current=0;
    inquireCurrent(node);
    msleep(15);
    while (sertialPort1->waitForReadyRead(TIMEOUT))
    {
        QByteArray temp=sertialPort1->readAll();
        receivedData.append(temp);
        for (int i=1;i<receivedData.size(); i++ )
        {
            if ( (int(receivedData.at(i-1))==13 )&& (int(receivedData.at(i))==10)  )
            {
               // curr_current=atoi( receivedData.toStdString().c_str() );
               //curr_current = receivedData.toInt();
                QString DataAsString = QTextCodec::codecForMib(106)->toUnicode(receivedData);
                curr_current=atof(DataAsString.toStdString().c_str());
                receivedData.clear();

                //int endTime=sysTime->elapsed();
                //int timeElapse= endTime - startTime;
                //cout<< "Motor"<< node<<" position:" << curr_temp<< "      Time elapsed:"<<timeElapse<< "ms"<<endl;
                return curr_current;
            }
        }
        msleep(3);
    }
    cout<< "get current timeout: " << node <<endl;
    return TIMEOUT_INDEX;
}

bool    Faulharbermotor::controlLoop()
{
    // data exchange in manual control mode
    if (status==0)
    {
        for (int i=0;i<NODENO;i++)
        {
            // get position and temprature and current with 10Hz
//            if ( sysTime->elapsed()-prevDataExchangeTime >100)
//            {
//                prevDataExchangeTime=sysTime->elapsed();
//                msrTem[i]=getTemprature(i);
//                msleep(10);
                msrPos[i]=getPosition(i);
                msleep(20);
//                msrCur[i]=getCurrent(i);
//                msleep(20);
//            }
        }
    }

    // moto 0 move forward
    if (status==1)
    {
        setPositionWithTargetReachNitofy(absClosePos, Motor0);
        status=2;
        cout<<"status2: approching close pos "<<endl;
				return true;
    }
    // motor 0 reach stop
    else if (status==2)
    {
       if ( targetReached(5000) )
       {
           msrPos[Motor0]=absClosePos;
           status=3;
           msleep(400);
           cout<<"status3: close pos reached"<<endl;
					 return true;
       }
       else
       {
           cout<<"close pos is not reached, timeout"<<endl;
           cout<<"jump to status 5"<<endl;
           status=5;
       }
    }
    //motor 1 rotate
    else if (status==3)
    {   
        if (needleDir==1)
        {
            setRelativePositionWithTargetReachNitofy(absLockPos, Motor1);
        }
        else
        {
            setRelativePositionWithTargetReachNitofy(-absLockPos, Motor1);
        }
        status=4;
        cout<<"status4: start locking "<<endl;
        //cout<<"status4: start locking "<<endl;
        cout << "Time: " << sysTime->elapsed() << std::endl;
				return true;

    }

    else if (status==4)
    {
        if ( targetReached(5000) )
        {
            status=5;
            msleep(10);
            cout<<"status5: lock pos reached"<<endl;
                    cout << "Time: " << sysTime->elapsed() << std::endl;
						return true;
        }
        else
        {
            status=52;
            cout<<"lock pos is not reached, timeout"<<endl;
        }
    }
    else if (status==5 )
    {
        setPositionWithTargetReachNitofy(absOpenPos, Motor0);
        msleep(10);
        status=6;
        cout<<"status6: appraching open pos"<<endl;
				return true;
    }
    else if (status==6)
    {
        if (targetReached(5000) )
        {
            msrPos[Motor0]=absOpenPos;
            status=9;
            msleep(10);
            cout<<"status7: open pos reached"<<endl;
						return true;
        }
        else
        {
            cout<<"open pos is not reached, timeout"<<endl;
						
        }
    }
    else if (status==9)
    {
        // update cmdPos to MsrPos when finish stitching
        status=0;
        static int count = 1;
        cout<<"stitch finished" << ", #" << count++ <<endl;

        cout<<"\n\n\n"<<endl;
        needleDir=-needleDir;
				return true;
    } 
				
		return false;
}

void    Faulharbermotor::inquireCurrent(int node)
{
    char ComPos[10];
    sprintf(ComPos, "%dGRC\n", node);
    sertialPort1->write(ComPos);
}

void    Faulharbermotor::inquirePos(int node)
{
    char ComPos[10];
    sprintf(ComPos, "%dPOS\n", node);
    sertialPort1->write(ComPos);
}

void    Faulharbermotor::inquireTemprature(int node)
{
    char ComPos[10];
    sprintf(ComPos, "%dTEM\n", node);
    sertialPort1->write(ComPos);
}

// successful:  return 1
// timeout:     return 0;
int     Faulharbermotor::readData(int &data)
{
    while (sertialPort1->waitForReadyRead(40))
    {
        QByteArray temp=sertialPort1->readAll();
        receivedData.append(temp);
        for (int i=1;i<receivedData.size(); i++ )
        {
            if ( (int(receivedData.at(i-1))==13 )&& (int(receivedData.at(i))==10)  )
            {
                //data=atof( receivedData.toStdString().c_str() );

                QString DataAsString = QTextCodec::codecForMib(106)->toUnicode(receivedData);
                data=atof(DataAsString.toStdString().c_str());
                //data = receivedData.toInt();
                receivedData.clear();
                return 1;
            }
        }
        msleep(8);
    }
    return 0;
}

void    Faulharbermotor::setPositionWithTargetReachNitofy(int pos,int node)
{
    char ComPos[30];
    sprintf(ComPos, "%dLA%d\nNP\n%dM\n", node, pos, node);
    sertialPort1->write(ComPos);
}

void    Faulharbermotor::setRelativePositionWithTargetReachNitofy(int pos,int node)
{
    char ComPos[30];
    sprintf(ComPos, "%dLR%d\nNP\n%dM\n", node, pos, node);
    sertialPort1->write(ComPos);
}

//target reached return 1
//never  reach return 0 after timeout
bool   Faulharbermotor::targetReached(int timeout)
{
    string cmd;
		//cout<<"temp "<<sertialPort1->waitForReadyRead(timeout)<<endl;
    while (sertialPort1->waitForReadyRead(timeout))
    {
        QByteArray temp=sertialPort1->readAll();
        receivedData.append(temp);

        for (int i=1;i<receivedData.size(); i++ )
        {

                //cmd=receivedData.toStdString();

                QString DataAsString = QTextCodec::codecForMib(106)->toUnicode(receivedData);

                receivedData.clear();
                return true;
        }
        msleep(10);
    }
    return false;
}

