#include "DynaMotorThread.h"

DynaMotorThread::DynaMotorThread()
{
    baudnum = 1;
    GoalPos[0]=0,GoalPos[1]=10;
    index = 0;
    deviceIndex = 2; //Needle driver USB

    cout<< "\n\nRead/Write example for Linux\n\n" <<endl;
    ///////// Open USB2Dynamixel ////////////
    if( dxl_initialize(deviceIndex, baudnum) == 0 )
    {
        cout<< "Failed to open USB2Dynamixel!\n" <<endl;
        cout<< "Press Enter key to terminate...\n" <<endl;
    }
    else
        cout<< "Succeed to open USB2Dynamixel!\n" <<endl;
//    dxl_write_word( LOOPERSERVO, P_GOAL_POSITION_L, looperInit );
//    dxl_write_word( NEEDLESERVO, P_GOAL_POSITION_L, needleInit );
}
	
void DynaMotorThread::setPositionNeedle(int pos)
{
     dxl_write_word( NEEDLESERVO, P_GOAL_POSITION_L, pos );
}

void DynaMotorThread::setPositionLooper(int pos)
{
     dxl_write_word( LOOPERSERVO, P_GOAL_POSITION_L, pos );
}

void DynaMotorThread::setPositionMandrel(int pos)
{
     dxl_write_word( MANDRELSERVO, P_GOAL_POSITION_L, pos );
}

// Print communication result
void DynaMotorThread::PrintCommStatus(int CommStatus)
{
    switch(CommStatus)
    {
    case COMM_TXFAIL:
        cout<<"COMM_TXFAIL: Failed transmit instruction packet!\n"<<endl;
        break;

    case COMM_TXERROR:
        cout<<"COMM_TXERROR: Incorrect instruction packet!\n"<<endl;
        break;

    case COMM_RXFAIL:
        cout<<"COMM_RXFAIL: Failed get status packet from device!\n"<<endl;
        break;

    case COMM_RXWAITING:
        cout<<"COMM_RXWAITING: Now recieving status packet!\n"<<endl;
        break;

    case COMM_RXTIMEOUT:
        cout<<"COMM_RXTIMEOUT: There is no status packet!\n"<<endl;
        break;

    case COMM_RXCORRUPT:
        cout<<"COMM_RXCORRUPT: Incorrect status packet!\n"<<endl;
        break;

    default:
        cout<<"This is unknown error code!\n"<<endl;
        break;
    }
}

// Print error bit of status packet
void DynaMotorThread::PrintErrorCode()
{
    if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
        cout<<"Input voltage error!\n"<<endl;

    if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
        cout<<"Angle limit error!\n"<<endl;

    if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
        cout<<"Overheat error!\n"<<endl;

    if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
        cout<<"Out of range error!\n"<<endl;

    if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
        cout<<"Checksum error!\n"<<endl;

    if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
        cout<<"Overload error!\n"<<endl;

    if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
        cout<<"Instruction code error!\n"<<endl;
}

