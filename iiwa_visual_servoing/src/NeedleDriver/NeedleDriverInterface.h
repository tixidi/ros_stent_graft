#ifndef NeedleDriverInterface_H
#define NeedleDriverInterface_H
#include "NeedleDriverControl.h"
#include <QMainWindow>
#include <QGridLayout>
#include <QPushButton>
#include <QComboBox>
#include <QSlider>
#include <QDebug>

#include "SewingMachine/DynaMotorThread.h"

class NeedleDriverInterface: public QWidget
{
    Q_OBJECT
public:
    NeedleDriverInterface();
//    int getneedleDriverPos(int ref);
private:
    NeedleDriverControl *motionContrller;
    QPushButton *beginAndEnd;
    bool buttonStatus;
    QSlider *positionSlider;
    QSlider *positionSlider1;

    DynaMotorThread *driver0;
    DynaMotorThread *driver1;

    int looperInit, needleInit;
public slots:
    void openAndClose();
    void changePos(int pos=0);
    void changePos0(int pos=0);
    void changePos1(int pos=0);
};

#endif
