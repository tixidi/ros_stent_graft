#include "iiwaControl.h"

using namespace std;

iiwaControl::iiwaControl(int robotID)
{
    if (robotID == 0)
    {
        //params.hostname_ = "172.31.1.147"; // change for multiple iiwa
				params.hostname_ = "192.170.10.105"; // change for multiple iiwa
        params.localPort_ = 9875;
    }
    else
    {
        params.hostname_ = "192.170.10.106";
        params.localPort_ = 9876;
    }

    params.Id_ = "SuturingKuka";
    params.iiwaPort_ = 18000;

    params.comm_timeout_ = 1000;
    params.inital_timeout_ = 1000;
    params.plannerSleepTime_ = 3;
    params.plannerCycleTime_ = 5;

    params.maxVelocity_ = Erl::Vector6d(200, 200, 200, 20, 20, 20);
    params.maxAcceleration_ = Erl::Vector6d(200, 200, 200, 20, 20, 20);
    params.maxJointVelocity_ = Eigen::Matrix<double, 7, 1>();
    params.maxJointVelocity_ << 20,20,20,20,20,20,20;
    params.maxJointAcceleration_ = params.maxJointVelocity_;

//    if (robotID == 0)
//        iiwaConnected = false;
//    else
        iiwaConnected = iiwa.start(params);
    if (iiwaConnected)
        cout << params.hostname_ << endl;
    msleep(1000);
}


void iiwaControl::run()
{
//    iiwaPose = iiwa.getDestTransform();
//    while(1)
//    {
//        iiwa.setTransform(iiwaPose);
//        iiwa.waitForDestinationReached();
//    }
//    kuka.stop();
//    return 0;
}

Erl::Transformd iiwaControl::getiiwaPose()
{
    return (iiwa.getDestTransform());
}

void iiwaControl::setiiwaPose(Erl::Transformd &desiredPose)
{
//    iiwaPose.setRotation(desiredPose.getRotation());
//    iiwaPose.setTranslation(desiredPose.getTranslation());
    desiredPose = Erl::Transformd::nearestOrthogonal(desiredPose); //Sometimes trans is not orth
    iiwa.setTransform(desiredPose);
    iiwa.waitForDestinationReached();
}

bool iiwaControl::iiwaReached()
{
    iiwa.waitForDestinationReached();
    return true;
}

void iiwaControl::EnableGravityCompensation()
{
    iiwa.applyGravityCompensationSettings();
}

void iiwaControl::DisableGravityCompensation()
{
    iiwa.disableGravityCompensationSettings();
}

