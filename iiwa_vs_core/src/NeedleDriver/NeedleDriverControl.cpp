#include "NeedleDriverControl.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
NeedleDriverControl::NeedleDriverControl()
{// Needle driver USB
    struct PortSettings myComSettingCom1 = {BAUD9600,DATA_8,PAR_NONE,STOP_1,FLOW_OFF,500};
    Com1 = new QextSerialPort("/dev/ttyUSB0",myComSettingCom1);
    //Com1 = new QextSerialPort("/dev/ttyS0",myComSettingCom1);
    Com1 ->open(QIODevice::ReadWrite);

    Com1->write("EN\n");
    Com1->write("ANSW0\n");
    Position0=0;
    Position1=0;
}

void NeedleDriverControl::setMotor0(int temp)
{
     Position0=temp;
}
void NeedleDriverControl::setMotor1(int temp)
{
    Position1=temp;
}

void NeedleDriverControl::run()
{
    while (true)
    {

        char ComPos1[50];
        sprintf(ComPos1, "0LA%d\n1LA%d\n",Position0,Position1);
        Com1->write(ComPos1);//,sizeof(ComPos1));
        Com1->write("M\n");//,sizeof("2M\n6M\n"));
//        cout<< ComPos1<<endl;
        msleep(20);
//        int c = Position1;
//        cout <<"Position 1 " << c << " " << endl;

        //exit the loop if user press "Esc" key  (ASCII value of "Esc" is 27)
    }
}
void NeedleDriverControl::setNeedleDriver0Status(bool close)
{
    if (close==true)
    {
        for (int i=Position0;i<=motorMax;i=i+5 )
        {
            Position0=i;
            msleep(1);
        }
    }
    else
    {
        for (int i=Position0;i>=motorMin ;i=i-5 )
        {
            Position0=i;
            msleep(1);
        }
    }
}

void NeedleDriverControl::startAndEnd(bool status)
{
    if (status==true)
    {
       Com1->write("EN\n");
       //cout<<"motor enable"<<endl;
    }
    else
    {
       Com1->write("DI\n");
       //cout<<"motor disable"<<endl;
    }
}


void NeedleDriverControl::setNeedleDriver1Status(bool close)
{
    if (close==true)
    {
        for (int i=Position1;i<=motorMax;i=i+5 )
        {
            Position1=i;
            msleep(1);
        }
    }
    else
    {
        for (int i=Position1;i>=motorMin ;i=i-5 )
        {
            Position1=i;
            msleep(1);
        }
    }
}

void NeedleDriverControl::setMotorRange(int min, int max)
{
    motorMin=min;
    motorMax=max;
}

