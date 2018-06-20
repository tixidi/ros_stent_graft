#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "functions.h"
#include "definitions.h"
#include "Erl.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <exotica/Exotica.h>
#include <iiwa_test/iiwaState.h>

#include <fstream>
#include <iostream>

#include <ctime>
#include <chrono>
#include <unistd.h>


string write_traj(int iiwaNo, double *iiwa_start_trans_inRob, double *iiwa_end_trans_inRob);
void replace_aico_trajectory(double *iiwa0_start_joint_state, double *iiwa1_start_joint_state);
void initialise_aico_trajectory();
MatrixXd run();

using namespace std;
using namespace exotica;
//exotica variables
MatrixXd traj_solution;
//ros variables
bool iiwa0_connected = false;
bool iiwa1_connected = false;
bool iiwa0_reached = false;
bool iiwa1_reached = false;
bool exotica_complete = false;
bool iiwa0_desiredEEInRob_sent = false;
bool iiwa1_desiredEEInRob_sent = false;
//int iiwa0_currJoints_count = -1;
//int iiwa1_currJoints_count = -1;
int iiwa0_state_count = -1;
int iiwa1_state_count = -1;
double iiwa0_msrTransform[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
double iiwa1_msrTransform[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
double iiwa0_currJoints[7] = {0,0,0,0,0,0,0};
double iiwa1_currJoints[7] = {0,0,0,0,0,0,0};
double iiwa0_desiredEEInRob[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
double iiwa1_desiredEEInRob[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
double iiwa0_default_currJoints[7] = {0.4836, -0.8033, -0.1767, 1.6785, -2.7219, -1.6706, 1.9363};
double iiwa1_default_currJoints[7] = {-1.2385, 0.7513, 0.1361, -1.3966, -1.7155, -1.3462, 0.1125};
double iiwa0_default_desiredEEInRob[12] = {-0.18725492,  0.84542901, -0.50018916, -0.59726775,-0.97520191, -0.09886336,  0.19798807, -0.25512601, 0.11793113,  0.52485612,  0.84297739,  0.40726137};
double iiwa1_default_desiredEEInRob[12] = {0.3162202,  -0.384865,    0.86711691,  0.40079543, -0.93000121, -0.30626176,  0.20322061, -0.50430395, 0.18734986, -0.87068207, -0.45477166,  0.48586735};
double iiwa0_default_msrTransform[12] = {-0.18725492,  0.84542901, -0.50018916, -0.59726775,-0.97520191, -0.09886336,  0.19798807, -0.25512601, 0.11793113,  0.52485612,  0.84297739,  0.40726137};
double iiwa1_default_msrTransform[12] = {0.3162202,  -0.384865,    0.86711691,  0.40079543, -0.93000121, -0.30626176,  0.20322061, -0.50430395, 0.18734986, -0.87068207, -0.45477166,  0.48586735};


void posCallback_iiwa0_connected(const std_msgs::Bool::ConstPtr& msg){
	iiwa0_connected = msg->data;
}
void posCallback_iiwa1_connected(const std_msgs::Bool::ConstPtr& msg){
	iiwa1_connected = msg->data;
}
void posCallback_iiwa0_reached(const std_msgs::Bool::ConstPtr& msg){
	iiwa0_reached = msg->data;
    cout<<"iiwa0_reached "<<iiwa0_reached<<endl;
}
void posCallback_iiwa1_reached(const std_msgs::Bool::ConstPtr& msg){
	iiwa1_reached = msg->data;
}
void posCallback_iiwa0_desiredEEInRob(const std_msgs::Float64MultiArray::ConstPtr& msg){
	for(int i = 0; i < 12; i++){
		iiwa0_desiredEEInRob[i] = msg->data[i];
	}
}
void posCallback_iiwa1_desiredEEInRob(const std_msgs::Float64MultiArray::ConstPtr& msg){
	for(int i = 0; i < 12; i++){
		iiwa1_desiredEEInRob[i] = msg->data[i];
	}
}
void posCallback_iiwa0_desiredEEInRob_sent(const std_msgs::Bool::ConstPtr& msg){
	iiwa0_desiredEEInRob_sent= msg->data;
}
void posCallback_iiwa1_desiredEEInRob_sent(const std_msgs::Bool::ConstPtr& msg){
	iiwa1_desiredEEInRob_sent= msg->data;
}
/*
void posCallback_iiwa0_msrTransform(const std_msgs::Float64MultiArray::ConstPtr& msg){
	for(int i = 0; i < 12; i++){
		iiwa0_msrTransform[i] = msg->data[i];
    }
}
void posCallback_iiwa1_msrTransform(const std_msgs::Float64MultiArray::ConstPtr& msg){
	for(int i = 0; i < 12; i++){
		iiwa1_msrTransform[i] = msg->data[i];
    }
}
void posCallback_iiwa0_currJoints(const sensor_msgs::JointState::ConstPtr& msg){
    if (msg->header.seq-iiwa0_currJoints_count == 1){
        iiwa0_currJoints_count = msg->header.seq;
        for(int i = 0; i < 7; i++){
            iiwa0_currJoints[i] = msg->position[i];
        }
    }

}
void posCallback_iiwa1_currJoints(const sensor_msgs::JointState::ConstPtr& msg){
    if (msg->header.seq-iiwa1_currJoints_count == 1){
        iiwa1_currJoints_count = msg->header.seq;
        for(int i = 0; i < 7; i++){
            iiwa1_currJoints[i] = msg->position[i];
        }
    }
}*/
void posCallback_iiwa0_state(const iiwa_test::iiwaState::ConstPtr& msg){

    //cout<<iiwa0_state_count <<" "<<msg->header.seq<<" "<<iiwa0_reached<<endl;
        cout<<"iiwa0 state =  "<<iiwa0_state_count<<" "<<msg->header.seq<<endl;
    if (msg->header.seq-iiwa0_state_count == 1){
        iiwa0_state_count = msg->header.seq;
        //iiwa0_reached = msg->iiwaReached;
        //iiwa0_connected = msg->iiwaConnected;

        for(int i = 0; i < 7; i++){
            iiwa0_currJoints[i] = msg->jointState.data[i];
        }
        for(int i = 0; i < 12; i++){
            iiwa0_msrTransform[i] = msg->transform.data[i];
        }
    }
}
void posCallback_iiwa1_state(const iiwa_test::iiwaState::ConstPtr& msg){
    if (msg->header.seq-iiwa1_state_count == 1){
        iiwa1_state_count = msg->header.seq;
        //iiwa1_reached = msg->iiwaReached;
        //iiwa1_connected = msg->iiwaConnected;

        for(int i = 0; i < 7; i++){
            iiwa1_currJoints[i] = msg->jointState.data[i];
        }
        for(int i = 0; i < 12; i++){
            iiwa1_msrTransform[i] = msg->transform.data[i];
        }
    }
}



int main(int argc,char **argv)
{
	//checking for number of arugments
	if (argc<3){
		cerr<<"Invalid number of arguments"<<endl;
		cerr<<"Usage: [mode 0:kuka 1:test] [runRobotIdx 0:iiwa0 1:iiwa1 2:both iiwa] [sleep time(ms)]"<<endl;
		return 1;
	}
	int mode = int(stoi(argv[1])); //0:kuka 1:test
	int runRobotIdx = int(stoi(argv[2])); //0:iiwa0 1:iiwa1 2:both iiwa
	
    //double sleepTime = stod(argv[3]);
		
	ros::init(argc, argv, "exotica_collision_detection1");
    ros::NodeHandle nh;
	ros::Publisher pub_iiwa0_destJoints = nh.advertise<sensor_msgs::JointState>("iiwa0_destJoints", 100, true);
	ros::Publisher pub_iiwa1_destJoints = nh.advertise<sensor_msgs::JointState>("iiwa1_destJoints", 100, true);
	ros::Publisher pub_exotica_complete = nh.advertise<std_msgs::Bool>("exotica_complete", 100, true);
	//for testing purpose, does not publish in kuka mode
	ros::Publisher pub_iiwa0_reached = nh.advertise<std_msgs::Bool>("iiwa0_reached", 100, true);
	ros::Publisher pub_iiwa1_reached = nh.advertise<std_msgs::Bool>("iiwa1_reached", 100, true);
	
	
    ros::Subscriber sub_iiwa0_state = nh.subscribe("iiwa0_state", 1000, posCallback_iiwa0_state);
    ros::Subscriber sub_iiwa1_state = nh.subscribe("iiwa1_state", 1000, posCallback_iiwa1_state);
    ros::Subscriber sub_iiwa0_connected = nh.subscribe("iiwa0_connected", 1000, posCallback_iiwa0_connected);
    ros::Subscriber sub_iiwa1_connected = nh.subscribe("iiwa1_connected", 1000, posCallback_iiwa1_connected);
    ros::Subscriber sub_iiwa0_reached = nh.subscribe("iiwa0_reached", 1000, posCallback_iiwa0_reached);
    ros::Subscriber sub_iiwa1_reached = nh.subscribe("iiwa1_reached", 1000, posCallback_iiwa1_reached);
    ros::Subscriber sub_iiwa0_desiredEEInRob = nh.subscribe("iiwa0_desiredEEInRob", 1000, posCallback_iiwa0_desiredEEInRob);
    ros::Subscriber sub_iiwa1_desiredEEInRob = nh.subscribe("iiwa1_desiredEEInRob", 1000, posCallback_iiwa1_desiredEEInRob);
    ros::Subscriber sub_iiwa0_desiredEEInRob_sent = nh.subscribe("iiwa0_desiredEEInRob_sent", 1000, posCallback_iiwa0_desiredEEInRob_sent);
    ros::Subscriber sub_iiwa1_desiredEEInRob_sent = nh.subscribe("iiwa1_desiredEEInRob_sent", 1000, posCallback_iiwa1_desiredEEInRob_sent);
    //ros::Subscriber sub_iiwa0_msrTransform = nh.subscribe("iiwa0_msrTransform", 1000, posCallback_iiwa0_msrTransform);
    //ros::Subscriber sub_iiwa1_msrTransform = nh.subscribe("iiwa1_msrTransform", 1000, posCallback_iiwa1_msrTransform);
    //ros::Subscriber sub_iiwa0_currJoints = nh.subscribe("iiwa0_currJoints", 1000, posCallback_iiwa0_currJoints);
    //ros::Subscriber sub_iiwa1_currJoints = nh.subscribe("iiwa1_currJoints", 1000, posCallback_iiwa1_currJoints);
    ros::Rate rate(100);


	//initialise exotica
	initialise_aico_trajectory();

	Initializer solver, problem;
	string file_name, solver_name, problem_name;
		
	//initialise exotica
	Server::InitRos(std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~")));
	Server::getParam("ConfigurationFile", file_name);
	Server::getParam("Solver", solver_name);
	Server::getParam("Problem", problem_name);
	XMLLoader::load("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/aico_trajectory.xml", solver, problem, "MySolver", "MyProblem");
	HIGHLIGHT_NAMED("XMLnode", "Loaded from XML");

    // Initialize
	PlanningProblem_ptr any_problem = Setup::createProblem(problem);
	MotionSolver_ptr any_solver = Setup::createSolver(solver);										
	// Assign the problem to the solver
	any_solver->specifyProblem(any_problem);

	// If necessary, modify the problem after calling sol->specifyProblem()
	// e.g. set different rho:
	//
	UnconstrainedTimeIndexedProblem_ptr my_problem;
    if (any_problem->type() == "exotica::UnconstrainedTimeIndexedProblem")
    {
		my_problem = std::static_pointer_cast<UnconstrainedTimeIndexedProblem>(any_problem);

		for (int t = 0; t < my_problem->getT() - 1; t++)
		{
			// This sets the precision of all time steps BUT the last one to zero
			// This means we only aim to minimize the task cost in the last time step
			// The rest of the trajectory minimizes the control cost
			//problem->setRho("Frame", 0.0, t);
			my_problem->setRho("Frame0",1e5,t);
			my_problem->setRho("Frame1",1e5,t);
			my_problem->setRho("JointLimit",1e4,t);
		}
	
	}
	


/*	
	double iiwa0_start_trans_inRob[12] = 
	{-0.18725492,  0.84542901, -0.50018916, -0.59726775,
	-0.97520191, -0.09886336,  0.19798807, -0.25512601,
	0.11793113,  0.52485612,  0.84297739,  0.40726137};
	double iiwa0_end_trans_inRob[12] = 
	{-0.21033985,  0.46070299, -0.86227241, -0.54119647,
	-0.95175919,  0.10504021,  0.28829324, -0.17398909,
	0.22338746,  0.88131296,  0.41638694,  0.56689833};
	write_traj(0,iiwa0_start_trans_inRob, iiwa0_end_trans_inRob);
	replace_aico_trajectory(iiwa0_default_currJoints,iiwa1_default_currJoints);
	run();
*/
	string target0, target1;
	bool complete_write_traj = false;
	exotica_complete = true;
	pub_exotica_complete.publish(exotica_complete);
/*	if (mode == 1){
		iiwa0_reached = true;
		iiwa1_reached = false;
		pub_iiwa0_reached.publish(iiwa0_reached);
		pub_iiwa1_reached.publish(iiwa1_reached);
    }*/
	
	rate.sleep();
	ofstream overall_traj_file;
	overall_traj_file.open("/home/charlie/Documents/workspace/ros_ws/overall_traj.txt");
	
	
	while(ros::ok){
        //cout<<iiwa0_reached<<" "<<iiwa0_desiredEEInRob_sent<<" "<<iiwa1_reached<<" "<<iiwa1_desiredEEInRob_sent<<endl;
		//pub_exotica_complete.publish(exotica_complete);
		ros::spinOnce();

		//if (mode == 1){
		//	pub_iiwa0_reached.publish(iiwa0_reached);
		//	pub_iiwa1_reached.publish(iiwa1_reached);
		//	rate.sleep();
		//}
		// Create the initial configuration

		if (mode == 0){
			if (runRobotIdx == 2 && iiwa0_connected && iiwa0_reached && iiwa1_connected && iiwa1_reached && iiwa0_desiredEEInRob_sent && iiwa1_desiredEEInRob_sent){
				target0 = write_traj(0, iiwa0_msrTransform, iiwa0_desiredEEInRob);
				target1 = write_traj(1, iiwa1_msrTransform, iiwa1_desiredEEInRob);
				//replace_aico_trajectory(iiwa0_currJoints,iiwa1_currJoints);
				//traj_solution = run();

				complete_write_traj = true;
				exotica_complete = false;
				pub_exotica_complete.publish(exotica_complete);
				rate.sleep();
			}
			if (runRobotIdx == 0 && iiwa0_connected && iiwa0_reached && !iiwa1_connected && !iiwa1_reached && iiwa0_desiredEEInRob_sent && !iiwa1_desiredEEInRob_sent){
				target0 = write_traj(0, iiwa0_msrTransform, iiwa0_desiredEEInRob);
				target1 = write_traj(1, iiwa1_default_msrTransform, iiwa1_default_desiredEEInRob);
				//replace_aico_trajectory(iiwa0_currJoints,iiwa1_default_currJoints);
				//traj_solution = run();
			/*	
				cout<<"iiwa0_currJoints"<<endl;
				for(int i = 0; i < 7; i ++){
					cout<<iiwa0_currJoints[i]<<" ";
				}
				cout<<endl;
				
				cout<<"iiwa0_desiredEEInRob"<<endl;
				for(int i = 0; i < 12; i ++){
					cout<<iiwa0_desiredEEInRob[i]<<" ";
				}
				cout<<endl;				
				*/
				for (int i = 0; i < 7; i ++){
					iiwa1_currJoints[i] = iiwa1_default_currJoints[i];
				}
				complete_write_traj = true;
				exotica_complete = false;
				pub_exotica_complete.publish(exotica_complete);
				rate.sleep();
			}
			if (runRobotIdx == 1 && !iiwa0_connected && !iiwa0_reached && iiwa1_connected && iiwa1_reached && !iiwa0_desiredEEInRob_sent && iiwa1_desiredEEInRob_sent){
				target0 = write_traj(0, iiwa1_default_msrTransform, iiwa0_default_desiredEEInRob);
				target1 = write_traj(1, iiwa1_msrTransform, iiwa1_desiredEEInRob);
				//replace_aico_trajectory(iiwa0_default_currJoints,iiwa1_currJoints);
				//traj_solution = run();
				for (int i = 0; i < 7; i ++){
					iiwa0_currJoints[i] = iiwa0_default_currJoints[i];
				}
				complete_write_traj = true;
				exotica_complete = false;
				pub_exotica_complete.publish(exotica_complete);
				rate.sleep();
			}
		}
		if (mode == 1){
			if (iiwa0_reached && iiwa1_reached && iiwa0_desiredEEInRob_sent && iiwa1_desiredEEInRob_sent){
				//cout<<"<-----------------------11"<<endl;
				target0 = write_traj(0, iiwa0_default_msrTransform, iiwa0_desiredEEInRob);
				//iiwa0_default_msrTransform = iiwa0_desiredEEInRob;
				std::copy(std::begin(iiwa0_desiredEEInRob), std::end(iiwa0_desiredEEInRob), std::begin(iiwa0_default_msrTransform));
				target1 = write_traj(1, iiwa1_default_msrTransform, iiwa1_desiredEEInRob);
				//iiwa1_default_msrTransform = iiwa1_desiredEEInRob;
				std::copy(std::begin(iiwa1_desiredEEInRob), std::end(iiwa1_desiredEEInRob), std::begin(iiwa1_default_msrTransform));
				complete_write_traj = true;
				exotica_complete = false;
				pub_exotica_complete.publish(exotica_complete);
				rate.sleep();
			}
			if (iiwa0_reached && !iiwa1_reached && iiwa0_desiredEEInRob_sent && !iiwa1_desiredEEInRob_sent){
				/*cout<<"<-----------------------10"<<endl;
				cout<<"iiwa0_default_msrTransform"<<endl;
				for(int i = 0; i <12; i ++){
					cout<<iiwa0_default_msrTransform[i]<<" ";
				}
				cout<<endl;
				cout<<"iiwa0_desiredEEInRob"<<endl;
				for(int i = 0; i <12; i ++){
					cout<<iiwa0_desiredEEInRob[i]<<" ";
				}
				cout<<endl;*/
				target0 = write_traj(0, iiwa0_default_msrTransform, iiwa0_desiredEEInRob);
				std::copy(std::begin(iiwa0_desiredEEInRob), std::end(iiwa0_desiredEEInRob), std::begin(iiwa0_default_msrTransform));
				target1 = write_traj(1, iiwa1_default_msrTransform, iiwa1_default_desiredEEInRob);
				complete_write_traj = true;
				exotica_complete = false;
				pub_exotica_complete.publish(exotica_complete);
				rate.sleep();
			}
			if (!iiwa0_reached && iiwa1_reached && !iiwa0_desiredEEInRob_sent && iiwa1_desiredEEInRob_sent){
				//cout<<"<-----------------------01"<<endl;
				target0 = write_traj(0, iiwa0_default_msrTransform, iiwa0_default_desiredEEInRob);
				target1 = write_traj(1, iiwa1_default_msrTransform, iiwa1_desiredEEInRob);
				std::copy(std::begin(iiwa1_desiredEEInRob), std::end(iiwa1_desiredEEInRob), std::begin(iiwa1_default_msrTransform));			
				complete_write_traj = true;
				exotica_complete = false;
				pub_exotica_complete.publish(exotica_complete);
				rate.sleep();
			}
			if (iiwa0_reached || iiwa1_reached){
				//replace_aico_trajectory(iiwa0_default_currJoints,iiwa1_default_currJoints);
				for (int i = 0; i < 7; i ++){
					iiwa0_currJoints[i] = iiwa0_default_currJoints[i];
					iiwa1_currJoints[i] = iiwa1_default_currJoints[i];
				}
			}
		}
		if (complete_write_traj){
			//any_solver->specifyProblem(any_problem);

			Eigen::VectorXd q = Eigen::VectorXd::Zero(any_problem->N);

			for (int i = 0; i < 7; i ++){
				q(i) = iiwa0_currJoints[i];
				q(i+7) = iiwa1_currJoints[i];
			}		
		
			my_problem->setStartState(q);
			my_problem->getScene()->removeTrajectory("Target0");
			my_problem->getScene()->removeTrajectory("Target1");
			my_problem->getScene()->addTrajectory("Target0", target0);
			my_problem->getScene()->addTrajectory("Target1", target1);
			any_solver->Solve(traj_solution);


			//cout<<"-------------------------------------------"<<endl;
			iiwa0_desiredEEInRob_sent = false;
			iiwa1_desiredEEInRob_sent = false;
			complete_write_traj = false;
			
			
			int i = 0;	
            cout<<"123"<<endl;
            ros::spinOnce();
            rate.sleep();
            cout<<iiwa0_state_count<<" "<<iiwa0_reached<<endl;
			while (i < traj_solution.rows()){
                ros::spinOnce();
                rate.sleep();
                //cout<<i<<" "<<iiwa0_state_count<<iiwa0_reached<<endl;
                //if (iiwa0_state_count == 1)
                    //cout<<(bool)iiwa0_reached<<endl;
                if (iiwa0_reached or iiwa1_reached){
                    sensor_msgs::JointState msg0, msg1;
                    for (int j = 0; j <7; j ++){
                        msg0.name.push_back("iiwa0 joint " + std::to_string(j));
                        msg0.position.push_back(traj_solution.row(i)[j]);
                        msg1.name.push_back("iiwa1 joint " + std::to_string(j));
                        msg1.position.push_back(traj_solution.row(i)[j+7]);
                    }
                    pub_iiwa0_destJoints.publish(msg0);
                    rate.sleep();
                    pub_iiwa1_destJoints.publish(msg1);
                    rate.sleep();
					iiwa0_reached = false;
					iiwa1_reached = false;
                    cout<<traj_solution.row(i)<<endl;
					overall_traj_file<<traj_solution.row(i)<<endl;
					i = i + 1;
                }
                /*
				overall_traj_file<<traj_solution.row(i)<<endl;
				sensor_msgs::JointState msg0, msg1;
				for (int j = 0; j <7; j ++){
					msg0.name.push_back("iiwa0 joint " + std::to_string(j));
					msg0.position.push_back(traj_solution.row(i)[j]);
					msg1.name.push_back("iiwa1 joint " + std::to_string(j));
					msg1.position.push_back(traj_solution.row(i)[j+7]);
				}				
				pub_iiwa0_destJoints.publish(msg0);
				pub_iiwa1_destJoints.publish(msg1);
				rate.sleep();	
                //usleep(sleepTime);
                usleep(500);
				i = i + 1;		
                */
				
                /*if (mode == 1){
					iiwa0_reached = true;
					iiwa1_reached = false;
					pub_iiwa0_reached.publish(iiwa0_reached);
					pub_iiwa1_reached.publish(iiwa1_reached);
					rate.sleep();
                }*/
			}
			for (int i = 0; i < 7; i ++){
				iiwa0_default_currJoints[i] = traj_solution.row(traj_solution.rows()-1)[i];
				iiwa1_default_currJoints[i] = traj_solution.row(traj_solution.rows()-1)[i+7];
			}
            /*if (mode == 1){
				iiwa0_reached = true;
				iiwa1_reached = false;
				pub_iiwa0_reached.publish(iiwa0_reached);
				pub_iiwa1_reached.publish(iiwa1_reached);
				rate.sleep();
            }*/
			exotica_complete = true;
			pub_exotica_complete.publish(exotica_complete);
			rate.sleep();
		}
		
	}

	// Clean up
    // Run this only after all the exoica classes have been disposed of!
    Setup::Destroy();
	return 0;
}


/*
MatrixXd run(){
	MatrixXd traj_solution;
	{
		Server::InitRos(std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~")));
		MotionSolver_ptr solver = XMLLoader::loadSolver("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/aico_trajectory.xml");
        Eigen::MatrixXd solution;
        Timer timer;

        solver->Solve(solution);
		
        //HIGHLIGHT("Finished solving in " << timer.getDuration() << "s. Solution [" << solution << "]");
		HIGHLIGHT("Finished solving in " << timer.getDuration() << "s.");
		ofstream traj_solution_file;
		traj_solution_file.open("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/traj_solution");
		traj_solution_file<<solution;
		traj_solution = solution;
    }
    
	return traj_solution;

}
*/

MatrixXd run(){



	
	

   Eigen::MatrixXd solution;
    //ROS_INFO_STREAM("Calling solve()");
//    {
//        ros::WallTime start_time = ros::WallTime::now();
        // Solve the problem using the AICO solver
//        any_solver->Solve(solution);
        //double time = ros::Duration(
        //                  (ros::WallTime::now() - start_time).toSec())
        //                  .toSec();
        //ROS_INFO_STREAM_THROTTLE(0.5, "Finished solving (" << time << "s)");
        //ROS_INFO_STREAM_THROTTLE(0.5, "Solution " << solution.row(solution.rows() - 1));
        return solution;
        

/*
        ros::Rate loop_rate(50.0);
        int t = 0;
        ROS_INFO_STREAM_THROTTLE(0.5, "Publishing states to rviz ...");
        int PlaybackWaitInterval = 10;
        while (ros::ok())
        {
            int i = 1;
            if (t == 0 || t == solution.rows() - 1) i = PlaybackWaitInterval;
            while (i-- > 0)
            {
                any_problem->getScene()->Update(solution.row(t).transpose());
                any_problem->getScene()->getSolver().publishFrames();

                ros::spinOnce();
                loop_rate.sleep();
            }
            t = t + 1 >= solution.rows() ? 0 : t + 1;
        }*/
//    }
    
}

void replace_aico_trajectory(double *iiwa0_start_joint_state, double *iiwa1_start_joint_state){
	std::ifstream aico_trajectory_if("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/aico_trajectory.xml");
	std::string content( (std::istreambuf_iterator<char>(aico_trajectory_if) ),(std::istreambuf_iterator<char>()));
	//cout<<content<<endl;
	//std::size_t pos = str.find("<StartState>"); 
	string first_half = content.substr(0,content.find("<StartState>")-1);
	string second_half = content.substr(content.find("</StartState>"));

	content = first_half + "\t<StartState>"+std::to_string(iiwa0_start_joint_state[0])+" "+std::to_string(iiwa0_start_joint_state[1])+" "+std::to_string(iiwa0_start_joint_state[2])+" "+std::to_string(iiwa0_start_joint_state[3])+" "+std::to_string(iiwa0_start_joint_state[4])+" "+std::to_string(iiwa0_start_joint_state[5])+" "+std::to_string(iiwa0_start_joint_state[6])+" "+std::to_string(iiwa1_start_joint_state[0])+" "+std::to_string(iiwa1_start_joint_state[1])+" "+std::to_string(iiwa1_start_joint_state[2])+" "+std::to_string(iiwa1_start_joint_state[3])+" "+std::to_string(iiwa1_start_joint_state[4])+" "+std::to_string(iiwa1_start_joint_state[5])+" "+std::to_string(iiwa1_start_joint_state[6])+second_half;
	
	ofstream aico_trajectory_of;
	aico_trajectory_of.open("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/aico_trajectory.xml");
	aico_trajectory_of<<content;
	aico_trajectory_of.close();
	
}

string write_traj(int iiwaNo, double *iiwa_start_trans_inRob, double *iiwa_end_trans_inRob){
	//ofstream iiwa_traj;
	//iiwa_traj.open(strdup(("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/iiwa_"+std::to_string(iiwaNo)+".traj").c_str()));
	
	
	Matrix4d rHee_start, rHee_end;
	for (int i=0; i<3; i++){
		for (int j=0; j<4; j++){
			rHee_start(i,j) = iiwa_start_trans_inRob[i*4+j];
		}
	}
	rHee_start(3,3) = 1;	


	for (int i=0; i<3; i++){
		for (int j=0; j<4; j++){
			rHee_end(i,j) = iiwa_end_trans_inRob[i*4+j];
		}
	}
	rHee_end(3,3) = 1;

	Matrix4d cHr = Functions::readTransformEigen(strdup(("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/iiwa"+std::to_string(iiwaNo)+"2CameraTransFile.txt").c_str()));	

	Matrix4d cHee_start = cHr*rHee_start;
	Matrix4d cHee_end = cHr*rHee_end;
	Erl::Transformd cHr_start = Functions::Eigen2Erl(cHee_start);
	Erl::Transformd cHr_end = Functions::Eigen2Erl(cHee_end);
	
	//cout<<"rHee_start"<<endl;
	//cout<<rHee_start<<endl;
	//cout<<"rHee_end"<<endl;
	//cout<<rHee_end<<endl;
	//cout<<"cHr"<<endl;
	//cout<<cHr<<endl;
	//cout<<"cHee_start"<<endl;
	//cout<<cHee_start<<endl;
	//cout<<"cHee_end"<<endl;
	//cout<<cHee_end<<endl;
/*		
	iiwa_traj<<"1\n"<<"2\t"<<"8"<<endl;
	iiwa_traj<<"0\t"<<std::to_string(cHr_start.getX()/1000.0)<<"\t"<<std::to_string(cHr_start.getY()/1000.0)<<"\t"<<std::to_string(cHr_start.getZ()/1000.0);
	iiwa_traj<<"\t"<<std::to_string(cHr_start.getQuaternion().x())<<"\t"<<std::to_string(cHr_start.getQuaternion().y())<<"\t"<<std::to_string(cHr_start.getQuaternion().z())<<"\t"<<std::to_string(cHr_start.getQuaternion().w())<<endl;
	iiwa_traj<<"10\t"<<std::to_string(cHr_end.getX()/1000.0)<<"\t"<<std::to_string(cHr_end.getY()/1000.0)<<"\t"<<std::to_string(cHr_end.getZ()/1000.0);
	iiwa_traj<<" "<<std::to_string(cHr_end.getQuaternion().x())<<"\t"<<std::to_string(cHr_end.getQuaternion().y())<<"\t"<<std::to_string(cHr_end.getQuaternion().z())<<"\t"<<std::to_string(cHr_end.getQuaternion().w())<<endl;
	*/
/*	
	cout<<"quat"<<endl;
	cout<<std::to_string(cHr_end.getX()/1000.0)<<"\t"<<std::to_string(cHr_end.getY()/1000.0)<<"\t"<<std::to_string(cHr_end.getZ()/1000.0);
	cout<<" "<<std::to_string(cHr_end.getQuaternion().x())<<"\t"<<std::to_string(cHr_end.getQuaternion().y())<<"\t"<<std::to_string(cHr_end.getQuaternion().z())<<"\t"<<std::to_string(cHr_end.getQuaternion().w())<<endl;
*/	
	
	//iiwa_traj.close();
	
	
	
	string out = "1\n2\t8\n0\t"+std::to_string(cHr_start.getX()/1000.0)+"\t"+std::to_string(cHr_start.getY()/1000.0)+"\t"+std::to_string(cHr_start.getZ()/1000.0)+"\t"+std::to_string(cHr_start.getQuaternion().x())+"\t"+std::to_string(cHr_start.getQuaternion().y())+"\t"+std::to_string(cHr_start.getQuaternion().z())+"\t"+std::to_string(cHr_start.getQuaternion().w())+"\n10\t"+std::to_string(cHr_end.getX()/1000.0)+"\t"+std::to_string(cHr_end.getY()/1000.0)+"\t"+std::to_string(cHr_end.getZ()/1000.0)+" "+std::to_string(cHr_end.getQuaternion().x())+"\t"+std::to_string(cHr_end.getQuaternion().y())+"\t"+std::to_string(cHr_end.getQuaternion().z())+"\t"+std::to_string(cHr_end.getQuaternion().w());
	
	return out;
}

void initialise_aico_trajectory(){
	std::ifstream aico_trajectory_if("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/aico_trajectory_bk_20180609.xml");
	std::string content( (std::istreambuf_iterator<char>(aico_trajectory_if) ),(std::istreambuf_iterator<char>()));
	
	ofstream aico_trajectory_of;
	aico_trajectory_of.open("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_sewing/stentgraft_sewing_planning/resources/aico_trajectory.xml");
	aico_trajectory_of<<content;
	aico_trajectory_of.close();

}



