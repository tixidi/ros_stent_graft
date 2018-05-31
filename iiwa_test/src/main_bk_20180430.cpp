#include <ros/ros.h>
#include <std_msgs/Float32.h> 
#include <kukasunrise.h>
#include <kukasunriseplanned.h>

using namespace std;

//int main()
//{
//    KukaSunrise kuka;
//    kuka.connect("172.31.1.147", 9876, 18000, 60000, 1000);
//    Erl::sleep_ms(2000);

//    Erl::Transformd T0 = kuka.getDestTransform();
//    Erl::Timer_ms timer;
//    while (timer.elapsedTime() < 5000)
//    {
//        double t = timer.elapsedTime();
//        double s = 100 * std::sin(2 * ERL_PI * t /5000);
//        double newZ = T0.getTranslation()[2] + s;
//        Erl::Vector3d Vtmp = T0.getTranslation();
//        Vtmp[2] = newZ;
//        Erl::Transformd newTransform(T0.getRotation(), Vtmp);
//        kuka.setTransform(newTransform);
//        Erl::sleep_ms(1);
//    }

//    Eigen::Matrix<double, 7, 1> J0 = kuka.getJoints();
//    timer = Erl::Timer_ms();
//    while (timer.elapsedTime() < 5000)
//    {
//        double t = timer.elapsedTime();
//        double s = Erl::toRadian(40.0) * std::sin(2 * ERL_PI * t /5000);

//        Eigen::Matrix<double, 7, 1> Jtmp = J0;
//        Jtmp[6] += s;
//        kuka.setJoints(Jtmp);
//        Erl::sleep_ms(1);
//    }

//    kuka.close();
//    return 0;
//}

float zVal = 0;

void posCallback(const std_msgs::Float32::ConstPtr& msg)
{
	if(msg->data){
		cout<<"set z value to "<<msg->data<<endl;
		//to do: set z value
		//zVal = msg->data;
		
	}

}

int main(int argc, char **argv)
{

		//initialise ros 
		ros::init(argc, argv, "move_kuka");
    ros::NodeHandle nh;
		ros::Subscriber sub = nh.subscribe("move_kuka_command", 1000, posCallback);
    srand(time(0));
    ros::Rate rate(10);

		//ros::spin();		
		
    KukaSunrisePlanned kuka;
    KukaSunrisePlanned::InitialParameters params;
    params.hostname_ = "172.31.1.147";
//    params.hostname_ = "192.170.10.106";
//    params.hostname_ = "173.0.1.147";
    params.Id_ = "SuturingKuka";
    params.iiwaPort_ = 18000;
    params.localPort_ = 9875;
//    params.localPort_ = 9876;
    params.comm_timeout_ = 1000;
    params.inital_timeout_ = 6000;
    params.plannerSleepTime_ = 3;
    params.plannerCycleTime_ = 5;

    params.maxVelocity_ = Erl::Vector6d(200, 200, 200, 20, 20, 20);
    params.maxAcceleration_ = Erl::Vector6d(200, 200, 200, 20, 20, 20);
    params.maxJointVelocity_ = Eigen::Matrix<double, 7, 1>();
    params.maxJointVelocity_ << 20,20,20,20,20,20,20;
    params.maxJointAcceleration_ = params.maxJointVelocity_;

    bool iiwaConnected = kuka.start(params);
    Erl::sleep_ms(2000);


    if (iiwaConnected == false)
    {
        cout << "iiwa is not connected!" << endl;
        return (0);
    }
    else
        cout << "iiwa " << params.hostname_ << " is connected!" << endl;

//    Erl::Transformd T0 = kuka.getDestTransform();

//    double s = -100 ;
//    double newZ = T0.getTranslation()[2] + s;
//    Erl::Vector3d Vtmp = T0.getTranslation();
//    Vtmp[2] = newZ;

//    Erl::Transformd newTransform(T0.getRotation(), Vtmp);
//    newTransform = Erl::Transformd::nearestOrthogonal(newTransform);

//    kuka.setTransform(newTransform);
//    Erl::sleep_ms(1);



    Erl::Transformd rTee = kuka.getDestTransform();
    Erl::Transformd rTee_ = rTee;
    Erl::Transformd eeTee_;



    double r = -15 * ERL_PI / 180;
    Erl::Rotmatd rRee = rTee.getRotation();
    Erl::Rotmatd rRee_, eeRee_;
    eeRee_ = Erl::Rotmatd::fromEuler_XYZ(0.0,0.0,r);
    eeTee_.setRotation(eeRee_);
		//ros::spinOnce();
		//eeTee_.setZ(zVal);
    eeTee_.setZ(0);

    rTee_ = rTee * eeTee_;

//    rRee_ = rRee * eeRee_;

    cout << "rTee " << rTee << endl;
    cout << "rTee_ " << rTee_ << endl;
    cout << "eeTee_ " << eeTee_ << endl;

//    rTee_.setRotation(rRee_);

    // -------------------------------------------
    kuka.setTransform(rTee_);
    kuka.waitForDestinationReached();
    //---------------------------------------------

//    while (timer.elapsedTime() < 5000)
//    {
//        double t = timer.elapsedTime();
//        double s = 100 * std::sin(2 * ERL_PI * t /5000);
//        double newZ = T0.getTranslation()[2] + s;
//        Erl::Vector3d Vtmp = T0.getTranslation();
//        Vtmp[2] = newZ;
//        Erl::Transformd newTransform(T0.getRotation(), Vtmp);
//        newTransform = Erl::Transformd::nearestOrthogonal(newTransform);
//        kuka.setTransform(newTransform);
//        Erl::sleep_ms(1);
//    }

//    Eigen::Matrix<double, 7, 1> J0 = kuka.getJoints();
//    timer = Erl::Timer_ms();
//    while (timer.elapsedTime() < 2000)
//    {
//        double t = timer.elapsedTime();
//        double s = Erl::toRadian(40.0) * std::sin(2 * ERL_PI * t /5000);

//        Eigen::Matrix<double, 7, 1> Jtmp = J0;
//        Jtmp[6] += s;
//        kuka.setJoints(Jtmp);
//        Erl::sleep_ms(1);
//        cout << timer.elapsedTime() << " " << Jtmp[6] << endl;
//    }



//    Erl::Transformd T0 = kuka.getDestTransform();
//    double newZ = T0.getTranslation()[2] + 200;
//    Erl::Vector3d Vtmp = T0.getTranslation();
//    Vtmp[2] = newZ;
//    Erl::Transformd newTransform(T0.getRotation(), Vtmp);
//    newTransform = Erl::Transformd::nearestOrthogonal(newTransform);
//    kuka.setTransform(newTransform);
//    kuka.waitForDestinationReached();

    cout << "Finished!" << endl;

    kuka.stop();
    return 0;
}

