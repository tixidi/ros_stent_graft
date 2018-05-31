#ifndef NeedleDriverControl_H
#define NeedleDriverControl_H

#include <QThread>
#include <fstream> 
#include <QTimer>
#include <QTime>
#include "qextserialport/qextserialport.h"

#include <cmath>
#include <QDataStream>

#define PI 3.141592653

using namespace std; 

class NeedleDriverControl: public QThread
{  
    Q_OBJECT  
private:
    int Position0;
    int Position1;
    QextSerialPort *Com1;
    int motorMin, motorMax;
public:
    NeedleDriverControl();
    void startAndEnd(bool status);
    void setMotor0(int temp);
    void setMotor1(int temp);
    void run();
    void setNeedleDriver0Status(bool close);
    void setNeedleDriver1Status(bool close);
    void setMotorRange(int min, int max);
    int  getposition0()
    {
        return Position0;
    }
    int getposition1()
    {
        return Position1;
    }
}; 
#endif 
