#ifndef Faulharbermotor_H
#define Faulharbermotor_H

#include <QThread>
#include <fstream> 
#include <QTime>
#include <QTimer>
#include <QtSerialPort/QSerialPort>
#include <cmath>
#include <QDataStream>

#define PI 3.141592653
using namespace std; 
#define Motor0 0
#define Motor1 1

class Faulharbermotor: public QThread
{  
    Q_OBJECT  
private:
    QSerialPort  *sertialPort1;
    QByteArray   receivedData;
    QTime *Mytime;
    QTimer *myTimer ;
    int timeInverval();
    int preTime;
    int readData(int &data);

    int footPadel;
    int footPadelOutPut;
    int pastFootPadel[3];
    int footTimeOut;
    int swtichFoot;

    //autoatic cotrol
    int status=0;
    int lineMRange=719+35;
    int rotMRange=91000;
    double stopMargin=0.96;
    double stopMarginL=0.98;
    int needleDir=1;
    int autoOn=1;

    //calibration
    QTimer *caliTimer ;

    //stop sig
    bool stopSig;
    int runStitchingSequence();

    bool enableSig;
    int pos1, pos2;
    int tempr1, tempr2;
    double desVeScale;
public:
    Faulharbermotor();
    void setVelocity(int vel, int node);
    void inquireCurrent(int node);
    void inquirePos(int node);
    void inquireTemprature(int node);
    void setRelativePosition(int pos, int node);
    void setInputFromFootPadel(int input);
    int  getRange(int node);
    void startAuto();
    void stopAuto();
    bool enableFootClutchSig;

    QTimer *singleStitchTimer;
    int getPos(int node);
public slots:
//    void controlTickSlot1();
    void controlTickClutch();
    void setM0position(int pos);
    void setM1position(int pos);
    void returnHome();

    void openJaws();
    void closeJaws();


    void lockUp();
    void lockDown();


    void CalibrateDevice();
    void startCalibration();

    void enable();

    void disable();
    void footCluthSwitch();

    void startSingleStitch();
    void controlTickSingleStitch();
    void enableDisenable();
    bool getMotorOnOffSig() {return enableSig;}
    int getmotor0Temprature() {return tempr1;}
    int getmotor1Temprature() {return tempr2;}
    void setDeviceSpeed(int scale);
}; 

#endif 
