#ifndef DYNAMOTORTHREAD_H
#define DYNAMOTORTHREAD_H
#include <qthread.h>
#include <stdio.h>
#include <termio.h>
#include <unistd.h>
#include "dynamixel.h"
#include<iostream>
using namespace std;

// Control table address
#define P_GOAL_POSITION_L	30
#define P_GOAL_POSITION_H	31
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_MOVING		46

// Defulat setting
#define DEFAULT_BAUDNUM		1 // 1Mbps
#define LOOPERSERVO		5
#define NEEDLESERVO		1
#define MANDRELSERVO	11
//#define LOOPERSERVO		5 1 2 3 4
//#define NEEDLESERVO		7
class DynaMotorThread: public QThread
{
    Q_OBJECT
public:
    DynaMotorThread();
    void PrintCommStatus(int CommStatus);
    void PrintErrorCode(void);
    //void run();
public slots:
    void setPositionLooper(int pos);
    void setPositionNeedle(int pos);
	void setPositionMandrel(int pos);
private:
    int baudnum;
    int GoalPos[2];
    int index ;
    int deviceIndex;
    int Moving, PresentPos;
    int CommStatus;

};

#endif 

