#include "MasterSlaveComuniThread.h"
#define NUMMARKERSPERFRAME 16
#define NUMMARKERS NUMMARKERSPERFRAME
#define NUMFLOATS_2D 5*2*NUMMARKERS
#define NUMFLOATS_3D NUMMARKERS*12
#define NUMCOLS 26
ofstream  saveFile8;
extern  double test1, test2, test3, test4, test5, test6, test7, test8;

struct MyStruct
{
// int test1; // frame number;
// int test2[NUMMARKERS]; // left/right frame
// int test3[NUMMARKERS]; // marker id number
// float test4[NUMFLOATS_2D]; // xy pos of center and 4 vertices
// float test5[NUMFLOATS_3D]; // marker pose
    float test1[NUMMARKERS][NUMCOLS];
};

MasterSlaveComuniThread::MasterSlaveComuniThread(QObject *parent)
{

    saveFile8.open("vision recording.txt");
    //local address and port
//    IP_Local="192.168.137.143";
    IP_Local="127.0.0.1";
    Port_Local=3000;
    //remote address
//    IP2_Remote="192.168.137.1";
    IP2_Remote="127.0.0.1";
    Port_Remote=3002;

    // create a QUDP socket
    udpSocket = new QUdpSocket(this);

    // The most common way to use QUdpSocket class is
    // to bind to an address and port using bind()
    // bool QAbstractSocket::bind(const QHostAddress & address,
    //     quint16 port = 0, BindMode mode = DefaultForPlatform)
    udpSocket->bind(QHostAddress(IP_Local),Port_Local);

    connect(udpSocket,SIGNAL(readyRead()),this,SLOT(recv_slot()));

    //set sending interval
//    QTimer*	sendTimer= new QTimer();

//    connect(sendTimer,SIGNAL(timeout()),this,SLOT(send_slot()));

//    sendTimer->start(0.2*1000);

}

int MasterSlaveComuniThread::Start_Communication()
{

    send_slot();

    return 1;
}

void MasterSlaveComuniThread::send_slot()
{
    MyStruct DataStruct;
//    DataStruct.test1 = counter;
//    DataStruct.test2[0] = 0;
//    DataStruct.test2[1] = 1;

//    DataStruct.test3[0] = 1;
//    DataStruct.test3[1] = 2;
//    DataStruct.test3[2] = 4;
//    DataStruct.test3[3] = 8;

//    DataStruct.test4[0] = 3.4;
//    DataStruct.test4[1] = 3.5;
//    DataStruct.test4[2] = 3.6;
//    DataStruct.test4[3] = 3.7;

//    DataStruct.test5[0] = 3.8;
//    DataStruct.test5[1] = 3.9;
//    DataStruct.test5[2] = 4.0;
//    DataStruct.test5[3] = 4.1;
    for (int i = 0; i < NUMMARKERS; i++)
    {
        for (int j = 0; j < NUMCOLS; j++)
        {
            DataStruct.test1[i][j] = (float)counter + i + (float)0.5*j;
        }
    }


    counter++;

    QByteArray buf;
    QDataStream s(&buf, QIODevice::WriteOnly);
    // The encoding is big endian by default, on all systems. You
    // can change it if you wish.
    if (false) s.setByteOrder(QDataStream::LittleEndian);


    for (int i = 0; i < NUMMARKERS; i++)
    {
        for (int j = 0; j < NUMCOLS; j++)
        {
            s << DataStruct.test1[i][j];
        }
    }

//    s << DataStruct.test1;

//    for (int i = 0; i < NUMMARKERS; i++)
//    {
//        s << DataStruct.test2[i];
//    }

//    for (int i = 0; i < NUMMARKERS; i++)
//    {
//        s << DataStruct.test3[i];
//    }

//    for (int i = 0; i < NUMFLOATS_2D; i++)
//    {
//        s << DataStruct.test4[i];
//    }

//    for (int i = 0; i < NUMFLOATS_3D; i++)
//    {
//        s << DataStruct.test5[i];
//    }

    udpSocket -> writeDatagram(buf,QHostAddress(IP2_Remote),Port_Remote);//QHostAddress::LocalHost
    qDebug() << "Message sent from: " << IP_Local;
    qDebug() << "Message port: " << Port_Local;
    qDebug() << endl;

//    char sendBuffer[256]={0};
//    sprintf(sendBuffer,"%8.6f,%8.6f,%8.6f,%8.6f,%8.6f,%8.6f,%8.6f,%8.6f,%8.6f,%8.6f$", 1, 2, 3.0, 4.0, 5.0,6.0, 7.0 ,8.0 ,9.0,10.0,11.0,12.0,13.0,14.0);
//    udpSocket -> writeDatagram((sendBuffer),256,QHostAddress(IP2_Remote),Port_Remote);//QHostAddress::LocalHost
}

void MasterSlaveComuniThread::recv_slot()
{
    QHostAddress sender;
    quint16 senderPort;

    MyStruct DataStructRecv;

    QByteArray datagram;

    do {
        datagram.resize(udpSocket->pendingDatagramSize());
        udpSocket->readDatagram(datagram.data(), datagram.size(),&sender, &senderPort);
    } while (udpSocket->hasPendingDatagrams());

    QDataStream in(&datagram, QIODevice::ReadOnly);

    for (int i = 0; i < NUMMARKERS; i++)
    {
        for (int j = 0; j < NUMCOLS; j++)
        {
            in >> DataStructRecv.test1[i][j];
        }
    }
        saveFile8   << test1<< ", "
                    << test2<< ", "
                    << test3 << ", "
                    << test4 << ", "
                    << test5 << ", "
                    << test6 << ", "
                    << test7 << ", ";


                   for (int i = 0; i < NUMMARKERS; i++)
                   {
                       for (int j = 0; j < NUMCOLS; j++)
                       {
                           saveFile8 << DataStructRecv.test1[i][j]<< ", ";
                       }
                   }
                    saveFile8<<endl;

//    in >> DataStructRecv.test1;

//    for (int i = 0; i < NUMMARKERS; i++)
//    {
//        in >> DataStructRecv.test2[i];
//    }

//    for (int i = 0; i < NUMMARKERS; i++)
//    {
//        in >> DataStructRecv.test3[i];
//    }

//    for (int i = 0; i < NUMFLOATS_2D; i++)
//    {
//        in >> DataStructRecv.test4[i];
//    }

//    for (int i = 0; i < NUMFLOATS_3D; i++)
//    {
//        in >> DataStructRecv.test5[i];
//    }

    // old
//    QByteArray receivebuf;
//    QDataStream sRecv(&receivebuf, QIODevice::ReadWrite);
//    udpSocket -> readDatagram((char*)&DataStructRecv, sizeof(DataStructRecv),&sender, &senderPort);

//    qDebug() << "Message from: " << sender.toString();
//    qDebug() << "Message port: " << senderPort;

//    for (int i = 0; i < NUMMARKERS; i++)
//    {
//        for (int j = 0; j < NUMCOLS; j++)
//        {
//            printf("%f, ", DataStructRecv.test1[i][j]);
//        }
//        printf("\n");
//    }
//    printf("\n");
//qDebug() << endl;

//    printf("%d, ", DataStructRecv.test1);

//    for (int i = 0; i < NUMMARKERS; i++)
//    {
//        printf("%d, ", DataStructRecv.test2[i]);
//    }
//    for (int i = 0; i < NUMMARKERS; i++)
//    {
//        printf("%d, ", DataStructRecv.test3[i]);
//    }
//    for (int i = 0; i < NUMFLOATS_2D; i++)
//    {
//        printf("%f, ", DataStructRecv.test4[i]);
//    }
//    for (int i = 0; i < NUMFLOATS_3D; i++)
//    {
//        printf("%f, ", DataStructRecv.test5[i]);
//    }
    //printf("\n");
    //qDebug() << endl;

    //printf("%d, %d, %f, %f, %d\n", DataStructRecv.test1,DataStructRecv.test2,DataStructRecv.test3,DataStructRecv.test4,DataStructRecv.test5);

}


