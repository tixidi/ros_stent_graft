#ifndef MAINWINDOWVISION_H
#define MAINWINDOWVISION_H
#include <QMainWindow>
#include <QGridLayout>
#include <QPushButton>
#include <QComboBox>
#include <QSlider>
#include"visualTrackingThread.h"

class mainWindowVision : public QWidget
{
    Q_OBJECT
public:
    mainWindowVision();

private:
      QImage Mat2QImage(const cv::Mat_<double> &src);
      QImage Mat2QImage(const cv::Mat3b &src);
      QLabel *myLabelLeft;
      QLabel *myLabelRight;
//    DynaMotorThread *motionContrller;
//    QPushButton *beginAndEnd;
//    bool buttonStatus;

//    QSlider *joint1Slider;
//    QSlider *joint2Slider;
//    QSlider *joint3Slider;
//    QSlider *SynchronizeSlider;
//    QSlider *boat;

//    static const int JOINTINIPOS_2=380;
//    static const int JOINTSTART_2=JOINTINIPOS_2-310;
//    static const int JOINTEND_2=JOINTINIPOS_2+60;
//    static const int JOINTMIDDLE_2=(JOINTSTART_2+JOINTEND_2)*2./5.;
//    //bigger arm never changed
//    static const int JOINTINIPOS_3=1800;
//    static const int JOINTSTART_3=JOINTINIPOS_3;
//    static const int JOINTEND_3=JOINTINIPOS_3+1200;
//    static const int JOINTMIDDLE_3=(JOINTSTART_3+JOINTEND_3)/2.;

public slots:
      void disPlayImages( stereoImages a);
//    void openAndClose();
//    void Slider2Changed(int pos=JOINTINIPOS_2);
//    void Slider3Changed(int pos=JOINTINIPOS_3);
//    void Synchronized (int pos);
};

#endif
