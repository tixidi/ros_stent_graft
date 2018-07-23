#include "mainWindow.h"

mainWindow::mainWindow()
{ 
		
    pathPlanner=new KukaMotionPlanning();
    pathPlanner->start(QThread::HighPriority);

    startButton= new QPushButton("click to open");
    connect (startButton, SIGNAL(clicked()),this,SLOT(clickStartButton()) );
    buttonStatus=false;
    opMode=0;

    QComboBox *selectModeList=new QComboBox();
    selectModeList->addItem("path planning");
    selectModeList->addItem("path recording");
    selectModeList->addItem("path following");
    connect (selectModeList, SIGNAL(currentIndexChanged(int)),this,SLOT(changeOpMode(int)) );

    QPushButton *firButton1= new QPushButton("start FRI");
    QPushButton *firButton2= new QPushButton("Apply GravityComp");
    QPushButton *firButton3= new QPushButton("Disable GravityComp");
    QPushButton *recordButton= new QPushButton("Record");


    connect (firButton1, SIGNAL(clicked()),pathPlanner,SLOT(plannerStart()) );
//    connect (firButton2, SIGNAL(clicked()),pathPlanner,SLOT(plannerStop()) );
//    connect (firButton3, SIGNAL(clicked()),pathPlanner,SLOT(plannerClose()) );

    connect (firButton2, SIGNAL(clicked()),pathPlanner,SLOT(enableGravy()) );
    connect (firButton3, SIGNAL(clicked()),pathPlanner,SLOT(disableGravy()) );

    connect (recordButton, SIGNAL(clicked()),pathPlanner,SLOT(recordforLiang()) );

    pannelDis  = new QTextEdit();
    QObject::connect(pathPlanner, SIGNAL(kukaInformAvaiable(kukaInformation)), this, SLOT(disPlayInfoSlot(kukaInformation)));

    QGridLayout *mainLayout =new QGridLayout(this);
    mainLayout->addWidget(selectModeList,0,0,1,1);
    mainLayout->addWidget(startButton,   1,0,1,1);
    mainLayout->addWidget(firButton1  ,  2,0,1,1);
    mainLayout->addWidget(firButton2    ,3,0,1,1);
    mainLayout->addWidget(firButton3,    4,0,1,1);
    mainLayout->addWidget (recordButton,    5,0,1,1);
    mainLayout->addWidget (pannelDis,     0,1,6,1);

    this->setLayout(mainLayout);

    //dock with simulation
    #ifdef GraphicsON
    KukaGraphic *kukaSimulator=new KukaGraphic();
    QObject::connect(pathPlanner, SIGNAL(kukaInformAvaiable(kukaInformation)), kukaSimulator, SLOT(updateModelStates(kukaInformation)));
    mainLayout->addWidget (kukaSimulator,    6,0,5,2);
    this->setLayout(mainLayout);
    this->setFixedSize(1400,800);
    #endif

}

void mainWindow::clickStartButton()
{
    buttonStatus=!buttonStatus;
    if (buttonStatus==true)
    {
        pathPlanner->setOpMode(opMode);
        pathPlanner->onAndOffButton(1);
        startButton->setText("click to stop");
    }
    if (buttonStatus==false)
    {
       pathPlanner->onAndOffButton(-1);
       startButton->setText("click to start");
    }
}

void mainWindow::changeOpMode(int mode)
{
    opMode=mode;
}

void mainWindow::disPlayInfoSlot(kukaInformation a)
{
    Vector7d joint1;
    joint1<< a.jointAngles1(0),  a.jointAngles1(1), a.jointAngles1(2), a.jointAngles1(3), a.jointAngles1(4), a.jointAngles1(5), a.jointAngles1(6);
    Vector7d joint2;
    joint2<< a.jointAngles2(0),  a.jointAngles2(1), a.jointAngles2(2), a.jointAngles2(3), a.jointAngles2(4), a.jointAngles2(5), a.jointAngles2(6);

    joint1=joint1*180/3.1415926;
    joint2=joint2*180/3.1415926;
    char buffer[500];
    sprintf(buffer,"robot0 Joint Pos\n%4.2f   %4.2f   %4.2f   %4.2f   %4.2f   %4.2f   %4.2f \nrobot1 Joint Pos\n%4.2f   %4.2f   %4.2f   %4.2f   %4.2f   %4.2f   %4.2f ",
            joint1(0),joint1(1),joint1(2),joint1(3),joint1(4),joint1(5),joint1(6),joint2(0),joint2(1),joint2(2),joint2(3),joint2(4),joint2(5),joint2(6));
    pannelDis->setText(buffer);
    this->update();
}

