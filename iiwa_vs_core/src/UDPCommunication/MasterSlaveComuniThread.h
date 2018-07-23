#ifndef MasterSlaveComuni_H
#define MasterSlaveComuni_H
#include <QThread>
#include <QCoreApplication>
#include <QTimer>
#include <QObject>
#include <QtNetwork>
#include <QDataStream>
#include <QUdpSocket>

#include <fstream>


#include "global.h"


//#include "definitions.h"
using namespace std; 
class MasterSlaveComuniThread: public QThread  
{  
    Q_OBJECT
private:
    QUdpSocket *udpSocket ;
    //QTimer timer;
    QString IP_Local,IP2_Remote;
    int Port_Local, Port_Remote;
    int counter;
public:
    MasterSlaveComuniThread(QObject *parent = 0);
    int Start_Communication();
public slots:
    void recv_slot();
    void send_slot();
};  

#endif 

