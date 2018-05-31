#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "Faulharbermotor.h"
#include "footPadel.h"

#include <iostream>
using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

//    void on_pushButton_5_clicked();

    void on_SuCaliButton_clicked();
    void on_SuOpenJawsButton_clicked();
    void on_SuCloseJawsButton_clicked();
    void on_SuSingleRunButton_clicked();
    void on_PierceButton_clicked();

    void UIUpdate();
    void on_LockUpButton_clicked();

    void on_SUTap_tabBarClicked(int index);

    void on_SUhorizontalSlider1_valueChanged(int value);

    void on_SUhorizontalSlider_2_valueChanged(int value);

    void on_LockDownButton_clicked();

    void on_MotorOnOffButton_clicked();

    void on_SUSpeedSlider_valueChanged(int value);

private:
    Ui::MainWindow *ui;
    Faulharbermotor *sutureDevice;
    QTimer *UIUpdateTimer;
    footPadel_philip *footPadel;
};

#endif // MAINWINDOW_H
