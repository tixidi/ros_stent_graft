#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    sutureDevice=new Faulharbermotor();
//    sutureDevice->start();
    // run UI update 20Hz
    QTimer *UIUpdateTimer=new QTimer();
    connect(UIUpdateTimer, SIGNAL(timeout()) ,  this,  SLOT( UIUpdate()) );
    UIUpdateTimer->start(50);
    //initial UI
    //ui->SUhorizontalSlider1->setRange (sutureDevice->abdClosePos,sutureDevice->absOpenPos);
    //ui->SUhorizontalSlider_2->setRange(sutureDevice->absLockUpPos,sutureDevice->abdLockDownPos);
    ui->SUSpeedSlider->setRange(50,100);
    ui->SUSpeedSlider->setValue( 60 );


    footPadel =new footPadel_philip();
    footPadel->start();

    connect(footPadel, SIGNAL(button2ClickSig()), sutureDevice, SLOT( runSingleStitch()) );
}

MainWindow::~MainWindow()
{
    delete ui;
}



void MainWindow::UIUpdate()
{    
    deviceInfomation disInfo= sutureDevice->getAllCtrlInfomation();
    double Pos0=disInfo.msrPos[0];
    double Pos1=disInfo.msrPos[1];
    double tem0=disInfo.msrTem[0];
    double tem1=disInfo.msrTem[1];
    double cur0=disInfo.msrCur[0];
    double cur1=disInfo.msrCur[1];

    QString info= QString("position0: ")+ QString::number(Pos0)
                 +QString("                             ")
                 +QString("temprature0: ")+ QString::number(tem0)
                 +QString("                             ")
                 +QString("current0: ")+ QString::number(cur0)
                 +QString("\n\n")
                 +QString("position1: ")+ QString::number(Pos1)
                 +QString("                             ")
                 +QString("temprature1: ")+ QString::number(tem1)
                 +QString("                             ")
                 +QString("current1: ")+ QString::number(cur1);
    ui->displayInfo->setText(info);


   if (disInfo.status==0)
   {
       ui->SuCaliButton->setStyleSheet("color: blue; background-color: green");

   }
   else
   {
       ui->SuCaliButton->setStyleSheet("color: blue; background-color: red");
   }


   if (disInfo.enableSig==0)
   {
       ui->MotorOnOffButton->setStyleSheet("color: blue; background-color: red");

   }
   else
   {
       ui->MotorOnOffButton->setStyleSheet("color: blue; background-color: green");
   }
}


void MainWindow::on_SuCaliButton_clicked()
{
    sutureDevice->runSingleStitch();
}


void MainWindow::on_SuOpenJawsButton_clicked()
{
     //sutureDevice->openJaws();
}

void MainWindow::on_SuCloseJawsButton_clicked()
{
    //sutureDevice->closeJaws();
}

void MainWindow::on_SuSingleRunButton_clicked()
{
    //sutureDevice->startSingleStitch();
}

void MainWindow::on_PierceButton_clicked() {
    sutureDevice->runPierceDeg(20.0);
}


void MainWindow::on_LockUpButton_clicked()
{
//    sutureDevice->lockUp();

}

void MainWindow::on_LockDownButton_clicked()
{
//     sutureDevice->lockDown();

}

void MainWindow::on_SUTap_tabBarClicked(int index)
{
//    if (index==1)
//    {
//        //set slider to actual motor pos
//        ui->SUhorizontalSlider1 ->setValue( sutureDevice->getMsrPos(0) );
//        ui->SUhorizontalSlider_2->setValue( sutureDevice->getMsrPos(1) );
//        sutureDevice->enable();
//    }
//    else
//    {

//    }
}

void MainWindow::on_SUhorizontalSlider1_valueChanged(int value)
{
//    sutureDevice->setCmdPos(value, 0);
//    cout<<"here1"<< value<<endl;
}

void MainWindow::on_SUhorizontalSlider_2_valueChanged(int value)
{
//    sutureDevice->setCmdPos(value, 1);
}

void MainWindow::on_MotorOnOffButton_clicked()
{
    sutureDevice->enableToggle();
}

void MainWindow::on_SUSpeedSlider_valueChanged(int value)
{
    double speed= double(value) /100.;
    sutureDevice->setSpeed( speed );
}
