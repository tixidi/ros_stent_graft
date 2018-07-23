#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <QGridLayout>
#include <QPushButton>
#include <QComboBox>
#include <QTextEdit>
#include "kukaMotionPlanning.h"
#include "KUKAControl/definitions.h"
#include "UDPCommunication/MasterSlaveComuniThread.h"
#include"config.h"


#ifdef  GraphicsON
#include "Simulator/KukaGraphic.h"
#endif

#ifdef  SewingMachineON
#include "SewingMachine/DynaMotorThread.h"
#endif

class mainWindow : public QWidget
{
    Q_OBJECT
public:
    mainWindow();
private:
    KukaMotionPlanning *pathPlanner;
    bool buttonStatus;
    int  opMode;
    QPushButton *startButton;
    QTextEdit *pannelDis;

public slots:
    void clickStartButton();
    void changeOpMode(int mode=0);
    void disPlayInfoSlot(kukaInformation);
};

#endif
