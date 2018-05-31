#ifndef FOOTPADEL_PHILIP_H
#define FOOTPADEL_PHILIP_H

#include <iostream>
#include <QThread>
#include <QKeyEvent>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <cstdio>

#define JOY_DEV "/dev/input/js0"
using namespace std;

class footPadel_philip: public QThread
{
    Q_OBJECT
private:
    int statusSig;
    int prevSig;
    int button2Click;
public:
    footPadel_philip();
    void run();
    int getStatus();
public slots:

signals:

    void button2ClickSig();
};

#endif
