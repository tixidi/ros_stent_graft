#include "Faulharbermotor.h"

Faulharbermotor::Faulharbermotor()
{
    // open commnication
//    sertialPort1 =new QSerialPort("Com3");
    sertialPort1 =new QSerialPort("/dev/ttyS0");
    sertialPort1->setBaudRate(QSerialPort::Baud115200);
    sertialPort1->open(QIODevice::ReadWrite);

    // reset
    for (int i=0;i <2;i++)
    {
        char ComPos2[15];
        sprintf(ComPos2, "%dHO\n",i);
        sertialPort1->write(ComPos2);
        char ComPos3[15];
        sprintf(ComPos3, "%dANSW0\n",i);
        sertialPort1->write(ComPos3);      
    }

   disable();

    char ComPos31[20];
    sprintf(ComPos31, "0SP%d\n",128);
    sertialPort1->write(ComPos31);
    char ComPos41[20];
    sprintf(ComPos41, "1SP%d\n",2500);
    sertialPort1->write(ComPos41);


    // set tracking model, which can achieve 300Hz communicaiton frequency
//    char ComPos1[20];
//    sprintf(ComPos1, "BINSEND1\n");
//    sertialPort1->write(ComPos1);
//    msleep(100);

    //set track channel 1
    char ComPos2[4];
    ComPos2[0]=200;
    ComPos2[1]=2;
    ComPos2[2]=47;
    ComPos2[3]=110;
    sertialPort1->write(ComPos2);
    msleep(300);
    //set track channel 2
    char ComPos3[4];
    ComPos3[0]=202;
    ComPos3[1]=200;
    ComPos3[2]=47;
    ComPos3[3]=110;
    sertialPort1->write(ComPos3);  
    msleep(300);

    // set control tick timer
    myTimer =new QTimer();
    connect(myTimer, SIGNAL(timeout()) ,  this,  SLOT( controlTickClutch()) );
    Mytime=new QTime();
    Mytime->start();

    // intial status
    footPadel=0;
    footPadelOutPut=0;
    swtichFoot=0;


    //automatic control parameters
    status=0;
    lineMRange=750;
    rotMRange=92000;
    stopMargin=0.96;
    stopMarginL=0.94;
    needleDir=1;

    //
    caliTimer =new QTimer();
    connect(caliTimer, SIGNAL(timeout()) ,  this,  SLOT( CalibrateDevice()) );

    //stop sig
    stopSig=false;
    enableFootClutchSig=false;
//    footPadelOutPut=1;

   singleStitchTimer=new QTimer();
   connect(singleStitchTimer, SIGNAL(timeout()) ,  this,  SLOT( controlTickSingleStitch()) );
    pos1=0;
    pos2=0;
    desVeScale=5;
    tempr1=0;
    tempr2=0;
}

void Faulharbermotor::openJaws()
{
    enable();
    char ComPos[20];
    sprintf(ComPos, "%dLA%d\n0M\n", 0, int(lineMRange));
    sertialPort1->write(ComPos);
    sertialPort1->flush();
    msleep(1);

//    sertialPort1->write(ComPos);
//    sertialPort1->flush();
//    msleep(1);
    cout<< "OpenJaws" <<endl;
}

void Faulharbermotor::closeJaws()
{
    enable();
    char ComPos[20];
    sprintf(ComPos, "%dLA%d\n0M\n", 0, 0);
    sertialPort1->write(ComPos);
    sertialPort1->flush();
    msleep(1);
}

void Faulharbermotor::startAuto()
{
     enable();
     enable();
     autoOn=1;
     myTimer->start(10);
}

void Faulharbermotor::stopAuto()
{      
    stopSig=true;
}

void Faulharbermotor::controlTickClutch()
{
    if (abs(footPadelOutPut)>0 && autoOn==1)
    {
        int tem_status=runStitchingSequence();
        if (tem_status==7)
        {
            status=0;
            autoOn=0;
            // only press stop and finish all sequenc, then stop
            if (stopSig==true)
            {
                myTimer->stop();
            }
        }
    }
    else if (abs(footPadelOutPut)==0)
    {
        autoOn=1;
        msleep(5);
    }
}

void Faulharbermotor::controlTickSingleStitch()
{
        int tem_status=runStitchingSequence();
        if (tem_status==9)
        {
            status=0;
            singleStitchTimer->stop();
            disable();
        }
}

void Faulharbermotor::startSingleStitch()
{
    enable();
    enable();
    singleStitchTimer->start(10);
}

void Faulharbermotor::setDeviceSpeed(int scale)
{
    desVeScale=scale;
}

int Faulharbermotor::runStitchingSequence()
{
    const double v1Cap=16;
    const double v2Cap=2000;
    const int interval=1000;
    // inq pos
    if (status==0)
    {
        inquirePos(Motor0);
        int data=0;
        int sig=readData(data);
        if (sig==1)
        {
            cout<<"status1: "<<endl;
            status=1;
        }
    }
    // moto 0 move forward
    if (status==1)
    {
//        setM0position(0);
        setVelocity(- (desVeScale*v1Cap/10)  ,0);
        status=2;
        cout<<"status2: "<<endl;
    }
    // motor 0 reach stop
    else if (status==2)
    {
        inquirePos(Motor0);
        int pos=0;
        int sig=readData(pos);
        if (sig==1 && pos < lineMRange*(1-stopMarginL))
        {
            cout<<"status3: " <<"pos0 : "<<pos<<endl;
            status=3;
        }
    }

    else if (status==3)
    {
        if (needleDir==1)
        {
            setM1position( -rotMRange );
        }
        else
        {
            setM1position(0 );
        }
        status=4;
    }
    else if (status==4)
    {
        inquirePos(Motor1);
        int pos=0;
        int sig=readData(pos);

        if (needleDir==1)
        {
            if (sig==1 && pos <- rotMRange*stopMargin )
            {

                cout<<"status512: "<<"pos1 : "<<pos<<endl;
                status=5;
            }
        }
        if (needleDir==-1 )
        {
            if (sig==1 && pos > -rotMRange*(1-stopMargin ) )
            {
                cout<<"status52: "<<"pos1 : "<<pos<<endl;
                status=5;
            }
        }
    }
    else if (status==5 )
    {
        //setM0position(lineMRange);
        setVelocity((desVeScale*v1Cap/10),0);
        status=6;
    }
    else if (status==6)
    {
        inquirePos(Motor0);
        int pos=0;
        int sig=readData(pos);
        if (sig==1 && pos > lineMRange*(stopMarginL) )
        {
            cout<<"status0: "<<"pos0 : "<<pos<<endl;
            status=7;
            needleDir=-needleDir;
        }
    }
    else if (status==7)
    {
        inquireTemprature(Motor0);
        int tempr=0;
        int sig=readData(tempr);
        if (sig==1)
        {
            tempr1=tempr;
            cout<<"motor0 temprature: "<< tempr1<<endl;
            status=8;
        }
    }
    else if (status==8)
    {
        inquireTemprature(Motor1);
        int tempr=0;
        int sig=readData(tempr);
        if (sig==1)
        {
            tempr2=tempr;
            cout<<"motor1 temprature: "<< tempr2<<endl;
            status=9;
        }
    }
    return status;
}


void Faulharbermotor::returnHome()
{
    enable();
    setM0position(0);
    setM1position(0);   
}


void Faulharbermotor::startCalibration()
{
    enable();
    caliTimer->start(10);
}


void Faulharbermotor::CalibrateDevice()
{
    static int c_status=0;
    const int motor0Speed=7;
    const int motor0StopCurrent=730;
    const int motor0FreeCurrent=80;
    static int backStoperPos=0;
    static int forwardStoperPos=0;


    const int motor1Speed=600;
    const int motor1StopCurrent=470;
    const int motor1FreeCurrent=40;
    static int upStoperPos=0;
    static int downStoperPos=0;

    const double caliCale=0.98;
    const double caliCaleRot=0.93;

    //find hard stoper for motor0
    //move back
    if (c_status==0)
    {
        //need 2 command to move
        setVelocity(motor0Speed,0);
        setVelocity(motor0Speed,0);
        c_status=1;
    }
    //enter critical
    else if (c_status==1)
    {
        inquireCurrent(Motor0);
        int curr=0;
        int sig=readData(curr);
        if (sig==1 && curr > motor0StopCurrent)
        {
            setVelocity(- motor0Speed*0.5, 0);
            c_status=2;
            cout<<"back move enter critical: " <<curr<<endl;
        }
    }
    //out critical
    else if (c_status==2)
    {
        inquireCurrent(Motor0);
        int curr=0;
        int sig=readData(curr);
        if (sig==1 && curr < motor0FreeCurrent)
        {
            setVelocity(0, 0);
            c_status=3;
            cout<<"out critical: " <<curr<<endl;
        }
    }
    //find back stoper posiiton
    else if (c_status==3)
    {
        inquirePos(Motor0);
        int pos=0;
        int sig=readData(pos);
        if (sig==1 )
        {
            cout<< "back stoper pos: "<< pos<<endl;
            backStoperPos=pos;
            c_status=4;
        }
    }

     //move forward
    else if (c_status==4)
    {
        setVelocity(-motor0Speed,0);
        c_status=5;
    }
    //enter critical
    else if (c_status==5)
    {
        inquireCurrent(Motor0);
        int curr=0;
        int sig=readData(curr);
        if (sig==1 && curr > motor0StopCurrent)
        {
            setVelocity( motor0Speed*0.5, 0);
            c_status=6;
            cout<<"forward move enter critical: " <<curr<<endl;
        }
    }
    //out critical
    else if (c_status==6)
    {
        inquireCurrent(Motor0);
        int curr=0;
        int sig=readData(curr);
        if (sig==1 && curr < motor0FreeCurrent)
        {
            setVelocity(0, 0);
            c_status=7;
            cout<<"out critical: " <<curr<<endl;
        }
    }
    //find forward stoper posiiton
    else if (c_status==7)
    {
        inquirePos(Motor0);
        int pos=0;
        int sig=readData(pos);
        if (sig==1 )
        {
            cout<< "forward stoper pos: "<< pos<<endl;
            forwardStoperPos=pos;
            c_status=16;
        }
    }

    //calibrate locking
    //move up
//    if (c_status==8)
//    {
//        setVelocity(- motor1Speed,1);
//        c_status=9;
//    }
//    //enter critical
//    else if (c_status==9)
//    {
//        inquireCurrent(Motor1);
//        int curr=0;
//        int sig=readData(curr);
//        if (sig==1 && curr > motor1StopCurrent)
//        {
//            setVelocity( motor1Speed*0.5, 1);
//            c_status=10;
//            cout<<"up move enter critical: " <<curr<<endl;
//        }
//    }
//    //out critical
//    else if (c_status==10)
//    {
//        inquireCurrent(Motor1);
//        int curr=0;
//        int sig=readData(curr);
//        if (sig==1 && curr < motor1FreeCurrent)
//        {
//            setVelocity(0, 1);
//            c_status=11;
//            cout<<"out critical: " <<curr<<endl;
//        }
//    }
//    //find back stoper posiiton
//    else if (c_status==11)
//    {
//        inquirePos(Motor1);
//        int pos=0;
//        int sig=readData(pos);
//        if (sig==1 )
//        {
//            cout<< "up stoper pos: "<< pos<<endl;
//            upStoperPos=pos;
//            c_status=12;
//        }
//    }

//     //move forward
//    else if (c_status==12)
//    {
//        setVelocity(motor1Speed,1);
//        c_status=13;
//    }
//    //enter critical
//    else if (c_status==13)
//    {
//        inquireCurrent(Motor1);
//        int curr=0;
//        int sig=readData(curr);
//        if (sig==1 && curr > motor0StopCurrent)
//        {
//            setVelocity( -motor1Speed*0.5, 1);
//            c_status=14;
//            cout<<"forward move enter critical: " <<curr<<endl;
//        }
//    }
//    //out critical
//    else if (c_status==14)
//    {
//        inquireCurrent(Motor1);
//        int curr=0;
//        int sig=readData(curr);
//        if (sig==1 && curr < motor1FreeCurrent)
//        {
//            setVelocity(0, 1);
//            c_status=15;
//            cout<<"out critical: " <<curr<<endl;
//        }
//    }
//    //find forward stoper posiiton
//    else if (c_status==15)
//    {
//        inquirePos(Motor1);
//        int pos=0;
//        int sig=readData(pos);
//        if (sig==1 )
//        {
//            cout<< "down stoper pos: "<< pos<<endl;
//            downStoperPos=pos;
//            c_status=16;
//        }
//    }

    else if (c_status==16 )
    {
        //calibration  done!
        caliTimer->stop();
        cout<< "\n\ncalibration is done!"<<endl;

        cout<<"downStoperPos :"<<downStoperPos<<endl;
        cout<<"upStoperPos: "<<upStoperPos<<endl;
        cout<<"forwardStoperPos:"<<forwardStoperPos<<endl;
        cout<<"backStoperPos: "<<backStoperPos<<endl;
        lineMRange= abs(forwardStoperPos - backStoperPos) *caliCale;
        rotMRange=  92000;//abs(downStoperPos - upStoperPos) *caliCaleRot;

        char ComPos3[15];
        sprintf(ComPos3, "%dHO\n",0);
        sertialPort1->write(ComPos3);
        char ComPos2[15];
        sprintf(ComPos2, "%dHO\n",1);
        sertialPort1->write(ComPos2);

//        // set range
//        char ComPos9[20];
//        sprintf(ComPos9, "%dLL%d\n",0,  -1);
//        sertialPort1->write(ComPos9);

//        char ComPos10[15];
//        sprintf(ComPos10, "%dLL%d\n",0, lineMRange );
//        sertialPort1->write(ComPos10);

//        char ComPos11[20];
//        sprintf(ComPos11, "%dLL%d\n",1,  1);
//        sertialPort1->write(ComPos11);

//        char ComPos12[15];
//        sprintf(ComPos12, "%dLL%d\n",1, -rotMRange );
//        sertialPort1->write(ComPos12);


        disable();
    }
}


void Faulharbermotor::lockUp()
{
    enable();
    setM1position(-rotMRange);
    msleep(1);
}
void Faulharbermotor::lockDown()
{
    enable();
    setM1position(0);
    msleep(1);
}


void Faulharbermotor::enable()
{
    enableSig=true;
    sertialPort1->write("EN\n");
    sertialPort1->write("EN\n"); // call twice: hack for ubuntu 16.04
}

void Faulharbermotor::disable()
{
    enableSig=false;
    sertialPort1->write("DI\n");
}

int Faulharbermotor::getRange(int node)
{
    if (node==0)
    {
        return lineMRange;
    }
    else
    {
        return rotMRange;
    }
}


void Faulharbermotor::setVelocity(int vel, int node)
{
    char ComPos[20];
    sprintf(ComPos, "%dV%d\n", node, vel);
    sertialPort1->write(ComPos);
    cout<< ComPos<<endl;
}

void Faulharbermotor::setRelativePosition(int pos, int node)
{
    char ComPos[20];
    sprintf(ComPos, "%dLR%d\nM\n", node, pos);
    sertialPort1->write(ComPos);
    cout<< ComPos<<endl;
}

void Faulharbermotor::setM0position(int pos)
{
    char ComPos[20];
    sprintf(ComPos, "%dLA%d\n0M\n", 0, pos);
    sertialPort1->write(ComPos);
    sertialPort1->flush();
    msleep(1);
    cout<< ComPos<<endl;
    pos1=pos;
}

void Faulharbermotor::setM1position(int pos)
{
    char ComPos[20];
    sprintf(ComPos, "%dLA%d\n1M\n", 1, pos);
    sertialPort1->write(ComPos);
    sertialPort1->flush();
    cout<< ComPos<<endl;
    msleep(1);
    pos2=pos;
}

int Faulharbermotor::getPos(int node)
{
    if (node==0)
    {
        return pos1;
    }
    else
    {
        return pos2;
    }
}

void Faulharbermotor::inquireCurrent(int node)
{
    char ComPos[10];
    sprintf(ComPos, "%dGRC\n", node);
    sertialPort1->write(ComPos);
    msleep(1);
//    cout<< ComPos<<endl;
}

void Faulharbermotor::inquirePos(int node)
{
    char ComPos[10];
    sprintf(ComPos, "%dPOS\n", node);
    sertialPort1->write(ComPos);
    msleep(1);
//    cout<< ComPos<<endl;
}

void Faulharbermotor::inquireTemprature(int node)
{
    char ComPos[10];
    sprintf(ComPos, "%dTEM\n", node);
    sertialPort1->write(ComPos);
    msleep(1);
    cout<< ComPos<<endl;
}

int Faulharbermotor::timeInverval()
{
    int currTime= Mytime->elapsed();
    int interval= Mytime->elapsed()- preTime;
    preTime=currTime;
//    cout<<"time:  "<<Mytime->elapsed()<<endl;
    return interval;
}

// successful read: reutnr 1
// timeoutL return 0;
int Faulharbermotor::readData(int &data)
{
    while (sertialPort1->waitForReadyRead(20))
    {
        QByteArray temp=sertialPort1->readAll();
        receivedData.append(temp);
        for (int i=1;i<receivedData.size(); i++ )
        {
            if ( (int(receivedData.at(i-1))==13 )&& (int(receivedData.at(i))==10)  )
            {
                receivedData.remove(  i- 1 , 2);
                data= receivedData.toInt();
                cout<<"time interal: "<< timeInverval()<< " data: " << data<<endl;
                receivedData.clear();
                return 1;
            }
        }
        msleep(1);
    }
    return 0;
}

// input from keyboard
void  Faulharbermotor::setInputFromFootPadel(int input)
{
   for (int i=0;i<2;i++)
   {
       pastFootPadel[2-i]=pastFootPadel[1-i];
   }
   pastFootPadel[0]=input;

   if (  pastFootPadel[1] ==1&&  pastFootPadel[0]==0  &&pastFootPadel[2] ==0 )
   {
       footPadel=0;
        swtichFoot=0;
   }

   if (  pastFootPadel[1] ==1&&  pastFootPadel[0]==1  &&pastFootPadel[2] ==1 )
   {
       footPadel=1;

   }

   if ( pastFootPadel[1] ==0&&  pastFootPadel[0]==0  &&pastFootPadel[2] ==0  )
   {
       if (swtichFoot==0)
       {
       footTimeOut=Mytime->elapsed();
       swtichFoot=1;
       //cout<<footTimeOut<<endl;
       }
   }

   if (  pastFootPadel[1] ==1&&  pastFootPadel[0]==0  &&pastFootPadel[2] ==1  )
   {
       footPadel=0;
   }

   footPadelOutPut=0;
   if (footPadel==1)
   {
       footPadelOutPut=1;
   }

}

void Faulharbermotor::footCluthSwitch()
{
//    enableFootClutchSig=!enableFootClutchSig;
//    if (enableFootClutchSig==false)
//    {
//       footPadelOutPut=1;
//    }
//    else
//    {
//       footPadelOutPut=0;
//    }
}


void Faulharbermotor::enableDisenable()
{
    enableSig=!enableSig;
    if (enableSig)
    {
        enable();
    }
    else
    {
        disable();
    }
}
