#include "mandrelDriverInterface.h"

MandrelDriverInterface::MandrelDriverInterface()
{
    this->setFixedSize(400,450);

	driver= new DynaMotorThread();
    driver->setPositionMandrel(0);

}

void MandrelDriverInterface::changePos(int pos)
{
    int driverpos;
    driverpos = pos; //-pos/25+240;
    driver->setPositionMandrel(driverpos);
}


