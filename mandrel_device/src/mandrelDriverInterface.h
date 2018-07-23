#ifndef MandrelDriverInterface_H
#define MandrelDriverInterface_H
#include <QMainWindow>
#include <QGridLayout>
#include <QPushButton>
#include <QComboBox>
#include <QSlider>
#include <QDebug>

#include "SewingMachine/DynaMotorThread.h"

class MandrelDriverInterface: public QWidget
{
    Q_OBJECT
public:
    MandrelDriverInterface();
private:
    QPushButton *beginAndEnd;
    bool buttonStatus;
    QSlider *positionSlider;
    QSlider *positionSlider1;

	DynaMotorThread *driver;

public slots:
    void changePos(int pos=0);
};

#endif
