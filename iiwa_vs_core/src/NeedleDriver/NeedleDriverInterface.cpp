#include "NeedleDriverInterface.h"

NeedleDriverInterface::NeedleDriverInterface()
{
    this->setFixedSize(400,450);

    driver0= new DynaMotorThread();
    driver0->setPositionNeedle(760);

    driver1= new DynaMotorThread();
    driver1->setPositionLooper(60);

//    motionContrller= new NeedleDriverControl();
//    motionContrller->start(QThread::NormalPriority);

    beginAndEnd= new QPushButton("click to open");
    connect (beginAndEnd, SIGNAL(clicked()),this,SLOT(openAndClose()) );
    buttonStatus=false;

    positionSlider=new QSlider(Qt::Horizontal);
    positionSlider->setTickInterval(1);
    positionSlider->setRange(0,4500);
    connect(positionSlider, SIGNAL(valueChanged(int)), this, SLOT(changePos0(int))  );
    // Motor changed ------
//    positionSlider=new QSlider(Qt::Horizontal);
//    positionSlider->setTickInterval(1);
//    positionSlider->setRange(0,4500);
    //connect(positionSlider, SIGNAL(valueChanged(int)), this, SLOT(changePos(int))  );


    positionSlider1=new QSlider(Qt::Horizontal);
    positionSlider1->setTickInterval(1);
    positionSlider1->setRange(0,4500);
    connect(positionSlider1, SIGNAL(valueChanged(int)), this, SLOT(changePos1(int))  );

//    motionContrller->setMotorRange(0, 4500);

    QGridLayout *mainLayout =new QGridLayout(this);
    mainLayout->addWidget(beginAndEnd);
    mainLayout->addWidget(positionSlider);
    mainLayout->addWidget(positionSlider1);
    this->setLayout(mainLayout);
    this->show();
}

void NeedleDriverInterface::openAndClose()
{
    buttonStatus=!buttonStatus;
    if (buttonStatus==true)
    {
//        motionContrller->setNeedleDriver0Status(true );
//        motionContrller->setNeedleDriver1Status(true );

        beginAndEnd->setText("click to stop");
//        motionContrller->startAndEnd(buttonStatus );
    }
    if (buttonStatus==false)
    {
//        motionContrller->setNeedleDriver0Status(false );
//        motionContrller->setNeedleDriver1Status(false );
        beginAndEnd->setText("click to start");
//        motionContrller->startAndEnd(buttonStatus );
    }
}

void NeedleDriverInterface::changePos(int pos)
{
//    motionContrller->setMotor1(pos);
    int test;
    positionSlider->setValue(pos);
    //cout << "DriverL " << pos << endl;
//    test = motionContrller->getposition1();
//    cout << " " << test << endl;
}

void NeedleDriverInterface::changePos0(int pos)
{
//    motionContrller->setMotor1( pos );
    positionSlider->setValue(pos);
    int driverpos0;
    driverpos0 = 760 - pos/6.5; //-pos/25+240;
    driver0->setPositionNeedle(driverpos0);
    //cout<< "DriverR " << driverpos <<endl;
}


void NeedleDriverInterface::changePos1(int pos)
{
//    motionContrller->setMotor1( pos );
    positionSlider1->setValue(pos );
    int driverpos;
//    driverpos = 60+pos/22.5; //-pos/25+240;
    driverpos = 70+pos/20; //20
    driver1->setPositionLooper(driverpos);
    //cout<< "DriverR " << driverpos <<endl;
}

//int NeedleDriverInterface::getneedleDriverPos(int ref)
//{
//    if (ref==0)
//    {
////       return  motionContrller->getposition0();
//    }
//    else
//    {
////       return  motionContrller->getposition1();
//    }


//}


