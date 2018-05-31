#include "forceSensor.h"

using namespace std;

forceSensor::forceSensor()
{
    msleep(2500); // We wait some ms to be sure about OptoPorts enumerated PortList
    fs_running = -1;

    OPort* portlist=ports.listPorts(true);

    if (ports.getLastSize()>0)
    {
        fs_running = 1;
        daq.open(portlist[0]);
        daq.zeroAll();
        cout << "Force Sensor running!" << endl;

        int size=daq.read(pack3D,false);
        std::cout<<"x: "<<pack3D.x<<" y: "<<pack3D.y<<" z: "<<pack3D.z<<std::endl;
    }
}

void forceSensor::run()
{
    while(1)
    {
        int size=daq.read(pack3D,false);
    }
}

int forceSensor::getX()
{
    int size=daq.read(pack3D,false);
    return pack3D.x;
}

int forceSensor::getY()
{
    int size=daq.read(pack3D,false);
    return pack3D.y;
}

int forceSensor::getZ()
{
    int size=daq.read(pack3D,false);
    return pack3D.z;
}

void forceSensor::zeroSensor()
{
    daq.zeroAll();
}
