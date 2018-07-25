#include "kukaMotionPlanning.h"

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include "sensor_msgs/JointState.h"
#include <iiwa_test/iiwaState.h>
#include <map>





void KukaMotionPlanning::posCallback_iiwa0_state(const iiwa_test::iiwaState::ConstPtr& msg){

    //cout<<iiwa0_state_count <<" "<<msg->header.seq<<" "<<iiwa0_reached<<endl;
//        cout<<"iiwa0 state =  "<<iiwa0_state_count<<" "<<msg->header.seq<<endl;
//    if (msg->header.seq-iiwa0_state_count == 1){
    if (msg->header.seq-iiwa0_state_count>=1){
        iiwa0_state_count = msg->header.seq;
        //iiwa0_reached = msg->iiwaReached;
        //iiwa0_connected = msg->iiwaConnected;

        //for(int i = 0; i < 7; i++){
        //    iiwa0_currJoints[i] = msg->jointState.data[i];
        //}

        for (int i = 0; i <3; i++){
            for (int j = 0; j <4; j++){
                    iiwa0_currentMartix4d(i,j) = msg->transform.data[4*i+j];
            }
        }
        for (int i = 0; i <3; i++){
            iiwa0_currentMartix4d(3,i) = 0;
        }
        iiwa0_currentMartix4d(3,3) = 1;

        iiwa0_currentTransformd = Functions::Eigen2Erl(iiwa0_currentMartix4d);
        iiwa0_msrTransform_received = true;

        int64_t nowms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        received_kuka0_msrTransform<<nowms<<" ";
        for (int i = 0; i <3; i++){
            for (int j = 0; j <4; j++){
                received_kuka0_msrTransform << iiwa0_currentMartix4d(i,j) <<" ";
            }
        }
        for (int i = 0; i <3; i++){
            for (int j = 0; j <4; j++){
                received_kuka0_msrTransform << iiwa0_currentTransformd(i,j) <<" ";
            }
        }
        received_kuka0_msrTransform << endl;
    }
}


void KukaMotionPlanning::posCallback_iiwa1_state(const iiwa_test::iiwaState::ConstPtr& msg){
//    if (msg->header.seq-iiwa1_state_count == 1){
    if (msg->header.seq-iiwa1_state_count>=1){
        iiwa1_state_count = msg->header.seq;
        //iiwa1_reached = msg->iiwaReached;
        //iiwa1_connected = msg->iiwaConnected;

        //for(int i = 0; i < 7; i++){
        //    iiwa1_currJoints[i] = msg->jointState.data[i];
        //}

        for (int i = 0; i <3; i++){
            for (int j = 0; j <4; j++){
                    iiwa1_currentMartix4d(i,j) =  msg->transform.data[4*i+j];
            }
        }
        for (int i = 0; i <3; i++){
            iiwa1_currentMartix4d(3,i) = 0;
        }
        iiwa1_currentMartix4d(3,3) = 1;

        iiwa1_currentTransformd = Functions::Eigen2Erl(iiwa1_currentMartix4d);
        iiwa1_msrTransform_received = true;


        int64_t nowms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        received_kuka1_msrTransform<<nowms<<" ";
        for (int i = 0; i <3; i++){
            for (int j = 0; j <4; j++){
                received_kuka1_msrTransform << iiwa1_currentMartix4d(i,j) <<" ";
            }
        }
        for (int i = 0; i <3; i++){
            for (int j = 0; j <4; j++){
                received_kuka1_msrTransform << iiwa1_currentTransformd(i,j) <<" ";
            }
        }
        received_kuka1_msrTransform << endl;
    }
}


void KukaMotionPlanning::posCallback_iiwa0_connected(const std_msgs::Bool::ConstPtr& msg){
	//TODO
	//if (msg->data!=NULL)
	iiwa0_connected = msg->data;
	cout<<iiwa0_connected<<endl;
}
void KukaMotionPlanning::posCallback_iiwa1_connected(const std_msgs::Bool::ConstPtr& msg){
	//TODO
	//if (msg->data!=NULL)
	iiwa1_connected = msg->data;
}

void KukaMotionPlanning::posCallback_iiwa0_reached(const std_msgs::Bool::ConstPtr& msg){
	//TODO
	iiwa0_reached = msg->data;
}
void KukaMotionPlanning::posCallback_iiwa1_reached(const std_msgs::Bool::ConstPtr& msg){
	//TODO
    iiwa1_reached = (bool)msg->data;
    //cout<<"iiwa1_reached "<<(bool)msg->data<<endl;
}


void KukaMotionPlanning::posCallback_exotica_complete(const std_msgs::Bool::ConstPtr& msg){
	//TODO
    exotica_complete = (bool)msg->data;
}

void KukaMotionPlanning::plannerCallback(const std_msgs::String::ConstPtr& msg)
{
    status = 20;
//    trajectFile = const_cast<char*>((SRC_FILES_DIR + "trajectories/" + msg->data).c_str());
    trajectFile = strdup((SRC_FILES_DIR + "trajectories/" + msg->data).c_str());
    cout << "char * " << const_cast<char*>((SRC_FILES_DIR + "trajectories/" + msg->data).c_str()) << endl;
    cout << "pathPlanner->trajectFile " << trajectFile << endl;
    trajIndex_pre = 0;
    trajIndex = 0;
    // clear all previous trajectories:
    myTracker->toolLTraj_tvec.clear();
    myTracker->toolRTraj_tvec.clear();
//    EEInRobot_0.clear();
//    traj_ToolLInMan.clear();
//    traj_DriverL.clear();
}




KukaMotionPlanning::KukaMotionPlanning()
{

    //Calibration files --------------------------------------------------------------------------------
    fname_cHr0 =  strdup((SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/iiwa02CameraTransFile.txt").c_str());
    fname_cHr1 = strdup((SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/iiwa12CameraTransFile.txt").c_str());
    //fname_eeHl = strdup((SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/ToolLInEE.txt").c_str());
		fname_eeHl = strdup((SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/SutureInEE.txt").c_str());
    fname_eeHr = strdup((SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/ToolRInEE.txt").c_str());
    fname_eeHs = strdup((SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/SutureInEE.txt").c_str());
    fname_cTi = strdup((SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/iiwa2CameraTransFile.txt").c_str());
    fname_eTm = strdup((SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/MandrelInEE.txt").c_str());
    fname_mHm_ = strdup((SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/mHm_.txt").c_str());
//    fname_thread_ori = "Documents/workspace/ros_ws/src/stentgraft_planning/iiwa_visual_servoing/src/VisionSystem/caliInfo/transformations/thread_pose_ori.txt";
//    fname_thread_new = "Documents/workspace/ros_ws/src/stentgraft_planning/iiwa_visual_servoing/src/VisionSystem/caliInfo/transformations/thread_pose_new.txt";
    fname_ee_mat = strdup((SRC_FILES_DIR+"VisionSystem/caliInfo/test/eePose_mat.txt").c_str());
    fname_robotHmarker_mat = strdup((SRC_FILES_DIR+"VisionSystem/caliInfo/test/marker2robot.txt").c_str());
    fname_cameraHmarker_mat = strdup((SRC_FILES_DIR+"VisionSystem/caliInfo/test/marker2camera.txt").c_str());
    fname_cameraHmarker_handeye_mat = strdup((SRC_FILES_DIR+"VisionSystem/caliInfo/test/marker2handeye.txt").c_str());
    fname_ee_robpos = strdup((SRC_FILES_DIR+"VisionSystem/caliInfo/test/eeRobotPosture.txt").c_str());
    fname_robotJoints0 = "Data/RobotJoint0.txt";

    status = 100;

    //initialise ros and setup nodes
    std::map<std::string, std::string> remaps;
    remaps["__name"] = "iiwa_vs_core";
    ros::init(remaps,"iiwa_vs_core");
    ros::NodeHandle nh;


    //publisher
    pub_iiwa0_desiredEEInRob = nh.advertise<std_msgs::Float64MultiArray>("iiwa0_desiredEEInRob", 100, true);
    pub_iiwa0_desiredEEInRob_sent = nh.advertise<std_msgs::Bool>("iiwa0_desiredEEInRob_sent", 100, true);
    pub_iiwa1_desiredEEInRob = nh.advertise<std_msgs::Float64MultiArray>("iiwa1_desiredEEInRob", 100, true);
    pub_iiwa1_desiredEEInRob_sent = nh.advertise<std_msgs::Bool>("iiwa1_desiredEEInRob_sent", 100, true);
    pub_iiwas_vs_reached = nh.advertise<std_msgs::Bool>("iiwas_vs_reached", 100, true);
    //pub_exotica_complete = nh.advertise<std_msgs::Bool>("exotica_complete", 100);
    //pub_iiwa1_desiredEEInRob = nh.advertise<std_msgs::Float64MultiArray>("iiwa1_desiredEEInRob", 100);
	
	
    //subscriber
    sub_iiwa0_state = nh.subscribe("iiwa0_msrTransform", 1000, &KukaMotionPlanning::posCallback_iiwa0_state,this);
    sub_iiwa0_reached = nh.subscribe("iiwa0_reached", 1000, &KukaMotionPlanning::posCallback_iiwa0_reached,this);
    sub_iiwa0_connected = nh.subscribe("iiwa0_connected", 1000, &KukaMotionPlanning::posCallback_iiwa0_connected,this);
    //sub_iiwa0_currJoints = nh.subscribe("iiwa0_currJoints", 1000, &KukaMotionPlanning::posCallback_iiwa0_currJoints,this);
    sub_iiwa1_state = nh.subscribe("iiwa1_msrTransform", 1000, &KukaMotionPlanning::posCallback_iiwa1_state,this);
    sub_iiwa1_reached = nh.subscribe("iiwa1_reached", 1000, &KukaMotionPlanning::posCallback_iiwa1_reached,this);
    sub_iiwa1_connected = nh.subscribe("iiwa1_connected", 1000, &KukaMotionPlanning::posCallback_iiwa1_connected,this);
    //sub_iiwa1_currJoints = nh.subscribe("iiwa1_currJoints", 1000, &KukaMotionPlanning::posCallback_iiwa1_currJoints,this);
    sub_exotica_complete = nh.subscribe("exotica_complete", 1, &KukaMotionPlanning::posCallback_exotica_complete,this);
    sub_fname_toolmandrel = nh.subscribe("fname_traj_toolmandrel", 1, &KukaMotionPlanning::plannerCallback,this);


    cout << " ros::Subscriber!" << endl;

    srand(time(0));
    ros::Rate rate(10);


//    iiwa
#ifdef iiwaOn
    //iiwaControl *tmp0 = new iiwaControl(0);
    //iiwas.push_back(tmp0);
	//rate.sleep();
	ros::spinOnce();
	cout<<iiwa0_connected<<endl;
	if (iiwa0_connected == true)	
    //if (iiwas[0]->iiwaConnected == true)
    {
        //iiwas[0]->start(QThread::TimeCriticalPriority);
        cout << "iiwas 0 is connected!" << endl;
    }
    else
        cout << "iiwa 0 is NOT connected!" << endl;

    //iiwaControl *tmp1 = new iiwaControl(1);
    //iiwas.push_back(tmp1);
    if (iiwa1_connected == true)
    //if (iiwas[1]->iiwaConnected == true)
    {
        //iiwas[1]->start(QThread::TimeCriticalPriority);
        cout << "iiwas 1 is connected!" << endl;
    }
    else
        cout << "iiwa 1 is NOT connected!" << endl;
#endif

//    dualArmPlanner=new DualArmRobot();
//    dualArmPlanner->start(QThread::HighestPriority);
//    onAndOff=0;
//    controlopMode=0;
//    QTimer*	samplingTimer= new QTimer();
//    connect(samplingTimer,SIGNAL(timeout()),this,SLOT(samplingKukaStares()));
//    samplingTimer->start(100);
//    timeCnt= new QTime();
//    timeCnt->start();

    #ifdef VisionSystemON
    myTracker=new VisualTrackingThread();
    myTracker->start(QThread::LowestPriority );
        #ifdef  StereoImagesDisplayON
            mainWindowVision *VisionWindow=new mainWindowVision();
            VisionWindow->show();
            cout << "VisionWindow->show();" << endl;
            qRegisterMetaType<stereoImages>("stereoImages");
            QObject::connect(myTracker, SIGNAL(imagesReady(stereoImages )), VisionWindow, SLOT(disPlayImages( stereoImages)));
        #endif


        // Initialise tools ----------------------------------------------
           vector<vector<int> > ToolsID;
           int arr_MandrelID[8] = {413, 321, 97, 819, 123, 221, 928, 62,};
           double mandrelRadius = 0.066;
           int arr_ToolID_L[5] = {390, 267, 184, 399, 437};
           int arr_ToolID_S[1] = {717};
           int arr_ToolID_R[5] = {289, 239, 189, 876, 654};
//           int arr_ToolID_R[5] = {918, 2, 384, 834, 993};

           vector<int> MandrelID (arr_MandrelID, arr_MandrelID + sizeof(arr_MandrelID) / sizeof(arr_MandrelID[0]) );
           vector<int> ToolID_L (arr_ToolID_L, arr_ToolID_L + sizeof(arr_ToolID_L) / sizeof(arr_ToolID_L[0]));
           vector<int> ToolID_S (arr_ToolID_S, arr_ToolID_S + sizeof(arr_ToolID_S) / sizeof(arr_ToolID_S[0]));
           vector<int> ToolID_R (arr_ToolID_R, arr_ToolID_R + sizeof(arr_ToolID_R) / sizeof(arr_ToolID_R[0]));

           float arr_markersSize[3] = {mandrelRadius * tan(M_PI/MandrelID.size()), 0.0109, 0.0109};
           //float arr_markersSize[3] = {mandrelRadius * tan(M_PI/MandrelID.size()), 0.008, 0.008};
           vector<float> MarkersSize (arr_markersSize, arr_markersSize + sizeof(arr_markersSize) / sizeof(arr_markersSize[0]));

           ToolsID.push_back(MandrelID);
           ToolsID.push_back(ToolID_S);
           ToolsID.push_back(ToolID_R);
           myTracker->initTools(MarkersSize, ToolsID);
           myTracker->DrawTool = false;

           myTracker->DrawHandEye = false;
           myTracker->markerInHandEye = cv::Mat::eye(4,4, CV_64F);

           initTool_bool = false;

           //Handeye update
           handeye_window = 10;
           robot0InCamera_hist.resize(handeye_window);
           robot1InCamera_hist.resize(handeye_window);
           handeye_count0 = 0;
           handeye_count1 = 0;
           myTracker->markerInHandEye_updated = cv::Mat::eye(4,4,CV_64F);

           // Needle Pose ----------------------------------
           ifstream f_streamlHt(strdup((SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/ToolLHNeedleEnd_9x17.txt").c_str()));
             for (int i=0; i<4; i++)
                 for (int j=0; j<4; j++)
                     {
                         double variable;
                         f_streamlHt >> variable;
                         iniToolLHNeedle_ei(i,j)=variable;
                     }

         ifstream f_streamrHt(strdup((SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/ToolRHNeedleEnd_9x17.txt").c_str()));
           for (int i=0; i<4; i++)
               for (int j=0; j<4; j++)
                   {
                       double variable;
                       f_streamrHt >> variable;
                       iniToolRHNeedle_ei(i,j)=variable;
                   }

           //Kalman filter ------------------
           ROBOTSLEEP = 20;
           TRAJSTEP = 2;
           CAMERALATENCY = 50;
           cnt_robothist = 0;
           R_kalman = cv::Mat::eye(6,6,CV_64F);
           Q_kalman = cv::Mat::eye(6,6,CV_64F);
           P_kalman = cv::Mat::eye(6,6,CV_64F);
           K_kalman = cv::Mat::eye(6,6,CV_64F);

           R_kalman.at<double>(0,0) = 0.1E-3;
           R_kalman.at<double>(1,1) = 0.1E-3;
           R_kalman.at<double>(2,2) = 0.1E-3;
           R_kalman.at<double>(3,3) = 0.04;
           R_kalman.at<double>(4,4) = 0.04;
           R_kalman.at<double>(5,5) = 0.04;

           Q_kalman.at<double>(0,0) = 0.5E-3;  //0.1E-3 + 1E-3;
           Q_kalman.at<double>(1,1) = 0.5E-3; //0.1E-3 + 1E-3;
           Q_kalman.at<double>(2,2) = 0.5E-3; //0.1E-3 + 1E-3;
           Q_kalman.at<double>(3,3) = 0.01 + 0.2;
           Q_kalman.at<double>(4,4) = 0.01 + 0.2;
           Q_kalman.at<double>(5,5) = 0.01 + 0.2;

           //R_kalman = R_kalman + Q_kalman;
           P_kalman = Q_kalman.clone();
           K_kalman = P_kalman/(P_kalman+R_kalman);
    #endif

    // Inspection
    inspection_clicked = false;

//    myNeedleDriver=new NeedleDriverInterface();

    ifstream fstream_cHr0(fname_cHr0);
    ifstream fstream_eeHl(fname_eeHl);
    ifstream fstream_cHr1(fname_cHr1);
    ifstream fstream_eeHr(fname_eeHr);

    CAMERA_H_ROBOT0 = Mat::eye(4,4,CV_64F);
      for (int i=0; i<4; i++)
          for (int j=0; j<4; j++)
              {
                  double variable;
                  fstream_cHr0 >> variable;
                  CAMERA_H_ROBOT0.at<double>(i,j)=variable;
                  CAMERA_H_ROBOT0.copyTo(cHr0_updated);
              }

    CAMERA_H_ROBOT1 = Mat::eye(4,4,CV_64F);
    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            {
                double variable;
                fstream_cHr1 >> variable;
                CAMERA_H_ROBOT1.at<double>(i,j)=variable;
                CAMERA_H_ROBOT1.copyTo(cHr1_updated);
             }

    EE_H_TOOLL = Mat::eye(4,4,CV_64F);
      for (int i=0; i<4; i++)
          for (int j=0; j<4; j++)
              {
                  double variable;
                  fstream_eeHl >> variable;
                  EE_H_TOOLL.at<double>(i,j)=variable;
              }

    EE_H_TOOLR = Mat::eye(4,4,CV_64F);
        for (int i=0; i<4; i++)
            for (int j=0; j<4; j++)
                {
                    double variable;
                    fstream_eeHr >> variable;
                    EE_H_TOOLR.at<double>(i,j)=variable;
                }

    // Initialise hand-eye history --------------
    Matrix4d robot0InCam_tmp = Functions::CVMat2Eigen(CAMERA_H_ROBOT0);
    Matrix4d robot1InCam_tmp = Functions::CVMat2Eigen(CAMERA_H_ROBOT1);
    robotPosture robotpos_l = Functions::convertHomoMatrix2RobotPosture(robot0InCam_tmp);
    robotPosture robotpos_r = Functions::convertHomoMatrix2RobotPosture(robot1InCam_tmp);
    for (int i=0; i<handeye_window; i++)
    {
        robot0InCamera_hist[i].setPosition(robotpos_l.getPosition());
        robot0InCamera_hist[i].setQuaternion(robotpos_l.getQuaternion());
        robot1InCamera_hist[i].setPosition(robotpos_r.getPosition());
        robot1InCamera_hist[i].setQuaternion(robotpos_r.getQuaternion());
    }

    // Initialization
    INIT_SEW = true;
    INIT_STATUS = true;

    // Current time for file name -------------------
    timer = time(0);   // get time now
    struct tm * now = localtime( & timer );

    strftime (millisecbuffer,80,"%Y %m %d %H %M %S ",now);
    strftime (buffer,80,"%Y-%m-%d-%H-%M",now);
    struct timeval tp;
    gettimeofday(&tp, NULL);  
		ms = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds

    dir_demo=SRC_FILES_DIR+"demonstration/";
    fname_traj_toolsInCam = dir_demo + "toolmandrel.repeat_" + string(buffer);
    fname_traj_EEInBase = dir_demo + "EEInRobots.repeat_" + string(buffer);
    fname_hist_HandEye0_ = dir_demo + "HandEye0_" + string(buffer);
    fname_traj_tool_Timer = dir_demo + "toolsTimer.txt_" + string(buffer);
    fname_traj_EEInBase_Timer = dir_demo + "EEInRobotsTimer.repeat_" + string(buffer);
		fname_kuka_joints_traj = dir_demo + "kukaJoints.txt_" + string(buffer);
    fname_joints_0 = dir_demo + "robot0joints_" + string(buffer);
    fname_kuka0Hee = dir_demo + "robot0Hee.txt";
    received_kuka0_msrTransform.open(SRC_FILES_DIR+"demonstration/received_kuka0_msrTransform_" + string(buffer));
    received_kuka1_msrTransform.open(SRC_FILES_DIR+"demonstration/received_kuka1_msrTransform_" + string(buffer));
    iiwa0_transformd_file.open (SRC_FILES_DIR+"demonstration/iiwa0_transformd.txt_" + string(buffer));
    iiwa1_transformd_file.open (SRC_FILES_DIR+"demonstration/iiwa1_transformd.txt_" + string(buffer));
		


    // Mandrel ----------
    Num_Slot = 12;
    Dist_Slot = 9; //9mm
    Curr_Slot = 5;

    SampleTime=1;

    NUM_CYCLE = 0;
}

void KukaMotionPlanning::run()
{

    while (true)
    {
        if (status < 100)
        {
         //demonstration function
//            cout << "trajectFile: " << trajectFile << endl;
            pathPlanning();
//            bimanualSewing();
        }
        if (status == 100)
        {

        }

        ros::spinOnce();

    }
}


void KukaMotionPlanning:: pathPlanning()
{
    ////////////////////////////////////////
    static int RunRobotIndex = RUNROBOT_index_sub;
//    status = 20;
    /*
     * 20: bimanual: read & plot trajectory     *
     * 21: bimanual: move robot 0 to pre-initial pose (move into camera view)
     * 22: bimanual: move robot 1 to pre-initial pose (move into camera view)
     * 23: compute robots' initial pose via vision
     * 24: Move robot 0 to initial poses
     * 25: Move robot 1 to initial poses
     * 26: For sewing: bimanual, execute both tools
     * 27: Detect needle pose, to 28     *
     * 28: bimanual: adapt 1L; read & plot trajectory
     * 29: Detect needle pose, save t     *
     * 30: move iiwa
     * 31: Show thread by force
     *
     * 40: pause and ask for human intervention
    */
    ////////////////////////////////////////

    static std::vector<robotPosture> EEInRobot_0, traj_ToolLInMan, EEInRobot_1, traj_ToolRInMan;
    static std::vector<float> traj_DriverL, traj_DriverR;
//    static int trajIndex=0;
//    static int trajIndex_pre = 0;

    // For re-compute trajectories-------------------
    static int Size_1L = 1;
    Size_1L = 3235;//1273;//3500; //4047;
    // ----------------------------------------------

//    f_ee_mat.open(fname_ee_mat);
//    f_marker2robot.open(fname_robotHmarker_mat);
//    f_marker2camera.open(fname_cameraHmarker_mat);
//    f_marker2handeye.open(fname_cameraHmarker_handeye_mat);
//    f_ee_pose.open(fname_ee_robpos);
//    f_robotJoints0.open(fname_robotJoints0);
//    fKukaJointsTraj.open(fname_kuka_joints_traj);


    // Read calibration files ------------------
    timer = time(0);   // get time now
    struct tm * now = localtime( & timer );
    char millisecbuffer [80];
    strftime (millisecbuffer,80,"%Y %m %d %H %M %S ",now);
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds

    if (trajectFile[0]=='\0')
        status == 100;

    if (status == 20)
    {// For sewing: bimanual, read trajectories
        cout << "Status = 20!" << endl;

        readToolsTrajinMandrelandDrivers(0, EEInRobot_0, traj_ToolLInMan, traj_DriverL, fname_cHr0, fname_eeHl, trajectFile);
        readToolsTrajinMandrelandDrivers(1, EEInRobot_1, traj_ToolRInMan, traj_DriverR, fname_cHr1, fname_eeHr, trajectFile);

        if (traj_ToolLInMan.size()>1)
        {
            cout<<"status = 20 traj_ToolLInMan[0] ";
            traj_ToolLInMan[0].print();
            cout<<"status = 20 EEInRobot_0 ";
            EEInRobot_0[0].print();

            // Plot trajectory -------------------------

            vector<cv::Point3f> man_traj, toolL_traj, toolR_traj, toolL_rvec, toolR_rvec;

            // Bidan debug -----------------
            readMandrelToolsDriversInCamera(trajectFile, man_traj, toolL_traj, toolR_traj, toolL_rvec, toolR_rvec);
            Mat cHm;
            myTracker->toolsTracker->toolPose2cvTrans(myTracker->toolsTracker->Tools[0], cHm);

            for (int i=0; i<toolR_traj.size(); i++)
            {
                myTracker->toolLTraj_tvec.push_back(toolL_traj[i]);
                myTracker->toolRTraj_tvec.push_back(toolR_traj[i]);
                myTracker->toolLTraj_rvec.push_back(toolL_rvec[i]);
                myTracker->toolRTraj_rvec.push_back(toolR_rvec[i]);
                //cout << i << ": " << myTracker->toolLTraj_tvec[i].x << endl;
            }

            cout<<"Reading " << myTracker->toolLTraj_tvec.size() << " data points!" <<endl;
            cout << "myTracker->toolLTraj_tvec " << myTracker->toolLTraj_tvec[0] << endl;
            cout << "cHm " << cHm << endl;

            myTracker->DrawTrajectoryToolL = true;
            myTracker->DrawTrajectoryToolR = true;

            #ifndef SimulationON

                if (initialise){
                    time_t t = time(0);   // get time now
                    struct tm * now = localtime( & t );
                    char buffer [80];
                    strftime (buffer,80,"%Y-%m-%d-%H-%M",now);

                    std::string toolsInCamFname, fEEInRobotsFname, fHandEye0Name, fHandEye1Name,
                                fToolTimerName, fEEInRobotsTimerFname, fRefTrajFname;
                    std::string fileDir=SRC_FILES_DIR+"demonstration/";
                    toolsInCamFname = fileDir + "toolmandrel_step"+std::to_string(TRAJSTEP)+".repeat_" + string(buffer);
                    fEEInRobotsFname = fileDir + "EEInRobots.repeat_" + string(buffer);
                    fToolTimerName = fileDir + "toolsTimer.txt_" + string(buffer);
                    fEEInRobotsTimerFname = fileDir + "EEInRobotsTimer.repeat_" + string(buffer);

                    toolsInCam.open(toolsInCamFname.c_str());
                    fEEInRobots.open(fEEInRobotsFname.c_str());
                    fEEInRobots_timer.open(fEEInRobotsTimerFname.c_str());

                    myTracker->initializeRecodeVideo(fileDir, buffer);
                    myTracker->toolsInCamTimer.open(fToolTimerName.c_str());

                    if (RunRobotIndex == 0 || RunRobotIndex == 2)
                    {
                        fHandEye0Name = fileDir + "HandEye0_" + string(buffer);
                        fHandEye0.open(fHandEye0Name.c_str());
                    }
                    if (RunRobotIndex == 1 || RunRobotIndex == 2)
                    {
                        fHandEye1Name = fileDir + "HandEye1_" + string(buffer);
                        fHandEye1.open(fHandEye1Name.c_str());
                    }
                }



            #endif

                if (initialise){
                    //if(iiwas[0]->iiwaConnected == true || iiwas[1]->iiwaConnected == true)
                    ros::spinOnce();

                    cout << "RUNROBOT_mode " << RUNROBOT_mode << endl;
                    if (RUNROBOT_mode == 1)
                    {

                    }
                    if(iiwa0_connected || iiwa1_connected || RUNROBOT_mode == 1)
                    {
                        if (RunRobotIndex == 0 || RunRobotIndex == 2)
                        {
                            status = 21;
                            cout << "Robots 0 start moving to initial pose!" << endl;
                        }
                        else if (RunRobotIndex == 1 )
                        {
                            status = 22;
                            cout << "Robots 1 start moving to initial pose!" << endl;
                        }
                        else
                        {
                            status = 100;
                        }
                    }
                    else
                    {
                        cout << "iiwa0 connect: " << iiwa0_connected << " iiwa1 connect: " << iiwa1_connected << endl;
                        status = 100;
                    }
                }
                else{
                        status = 23;
                    }


            }
        else
        {
            cout << "There is no points in the trajectory file!" << endl;
            status = 100;
        }

        cout << "Status = " <<status << " !"<< endl;


    }

    else if (status == 21)
    {
        // Move iiwa 0 to initial pose---------------------
        // Read initial pose in trajectory --------------
        robotPosture eeInrobot = EEInRobot_0[0];
		ros::spinOnce();
//        cout << "iiwa 0 pose " << iiwas[0]->getiiwaPose().getX() << " " << iiwas[0]->getiiwaPose().getY() << " " << iiwas[0]->getiiwaPose().getZ() << " "
//                             << iiwas[0]->getiiwaPose().getQuaternion().w() << " " << iiwas[0]->getiiwaPose().getQuaternion().x() << " " << iiwas[0]->getiiwaPose().getQuaternion().y() << " " << iiwas[0]->getiiwaPose().getQuaternion().z()
//                             << endl;
		ros::spinOnce();
        cout << "iiwa 0 pose " << iiwa0_currentTransformd.getX() << " " << iiwa0_currentTransformd.getY() << " " << iiwa0_currentTransformd.getZ() << " "
                             << iiwa0_currentTransformd.getQuaternion().w() << " " << iiwa0_currentTransformd.getQuaternion().x() << " " << iiwa0_currentTransformd.getQuaternion().y() << " " << iiwa0_currentTransformd.getQuaternion().z()
                             << endl;	

        cout << "EEInRobot0 [0] " << eeInrobot.getPosition().transpose() << " "
                                  << eeInrobot.getQuaternion().w() << " " << eeInrobot.getQuaternion().x() << " " << eeInrobot.getQuaternion().y() << " " << eeInrobot.getQuaternion().z()
                                  << endl;

        // Move iiwa to initial pose ---------------
        Erl::Transformd iiwaTransformd = Functions::RobotPosture2ErlTransformd(eeInrobot);

        //cout << "iiwa0Current" << endl << iiwas[0]->getiiwaPose() << endl;
        ros::spinOnce();
        cout << "iiwa0Current" << endl << iiwa0_currentTransformd << endl;
        cout << "iiwa0Transformd" << endl << iiwaTransformd << endl;

//        myTracker->DrawHandEye = true;

		ros::spinOnce();
        iiwa0_transformd_file<<iiwaTransformd(0,0)<<endl;
        iiwa1_transformd_file<<iiwaTransformd(0,0)<<endl;
        bool iiwa_reached = moveIiwa(0, iiwaTransformd);//move iiwa0 and check if rechead

        //iiwas[0]->setiiwaPose(iiwaTransformd);
        //if (iiwas[0]->iiwaReached())
        if (iiwa_reached)
        {
            cout << "Robot 0 moved to camera view!" << endl;
            if (RunRobotIndex == 2)
            {
                status = 22;
                cout << "Status = " <<status << " !"<< endl;
            }
            else if (RunRobotIndex == 0)
            {
                status = 23;
                cout << "Status = " <<status << " !"<< endl;
            }
            else
            {
                status = 100;
            }

        }
    }

    else if (status == 22)
    {
        // Move iiwa 1 to initial pose ------------------
        // Read initial pose in trajectory --------------
        robotPosture eeInrobot = EEInRobot_1[0];

//        cout << "iiwa 1 pose " << iiwas[1]->getiiwaPose().getX() << " " << iiwas[1]->getiiwaPose().getY() << " " << iiwas[1]->getiiwaPose().getZ() << " "
//                             << iiwas[1]->getiiwaPose().getQuaternion().w() << " " << iiwas[1]->getiiwaPose().getQuaternion().x() << " " << iiwas[1]->getiiwaPose().getQuaternion().y() << " " << iiwas[1]->getiiwaPose().getQuaternion().z()
//                             << endl;
		ros::spinOnce();
        cout << "iiwa 1 pose " << iiwa1_currentTransformd.getX() << " " << iiwa1_currentTransformd.getY() << " " << iiwa1_currentTransformd.getZ() << " "
                             << iiwa1_currentTransformd.getQuaternion().w() << " " << iiwa1_currentTransformd.getQuaternion().x() << " " << iiwa1_currentTransformd.getQuaternion().y() << " " << iiwa1_currentTransformd.getQuaternion().z()<< endl;	

        cout << "EEInRobot1 [0] " << eeInrobot.getPosition().transpose() << " "
                                  << eeInrobot.getQuaternion().w() << " " << eeInrobot.getQuaternion().x() << " " << eeInrobot.getQuaternion().y() << " " << eeInrobot.getQuaternion().z()
                                  << endl;

        // Move iiwa to initial pose ---------------
        Erl::Transformd iiwaTransformd = Functions::RobotPosture2ErlTransformd(eeInrobot);

		ros::spinOnce();
        //cout << "iiwa1Current" << endl << iiwas[1]->getiiwaPose() << endl;
        cout << "iiwa1Current" << endl << iiwa1_currentTransformd << endl;
        cout << "iiwa1Transformd" << endl << iiwaTransformd << endl;

//        myTracker->DrawHandEye = true;

		ros::spinOnce();
        //cout<<"iiwa reached "<<iiwa1_reached<<" exotica complete "<<exotica_complete<<endl;
        bool iiwa_reached = moveIiwa(1, iiwaTransformd);//move iiwa0 and check if rechead

        //iiwas[1]->setiiwaPose(iiwaTransformd);
        //if (iiwas[1]->iiwaReached())
        if(iiwa_reached)
        {
            cout << "Robot 1 moved to camera view!" << endl;
            status = 23; //23
            cout << "Status = " <<status << " !"<< endl;

        }

    }

    else if(status == 23)
    {
        static Matrix4d rob0inCam_mat4d, rob1inCam_mat4d;
        if (RunRobotIndex == 0 || RunRobotIndex == 2){
            rob0inCam_mat4d = Functions::CVMat2Eigen(cHr0_updated);
        }
        if (RunRobotIndex == 1 || RunRobotIndex == 2){
            rob1inCam_mat4d = Functions::CVMat2Eigen(cHr1_updated);
        }

        if (initialise){

            // compute robots' initial pose via vision
            ////////// Read all configs ///////////////
            // Read handeye ------------
            ifstream f_stream_r2c0(fname_cHr0), f_stream_r2c1(fname_cHr1);
            static Matrix4d rob0inCam_mat4d, rob1inCam_mat4d;
            for (int i=0; i<4; i++)
            {
                for (int j=0; j<4; j++)
                    {
                        f_stream_r2c0 >> rob0inCam_mat4d(i,j);
                        f_stream_r2c1 >> rob1inCam_mat4d(i,j);
                    }
            }
            cout << "rob0inCam_mat4d " << endl << rob0inCam_mat4d << endl;
            cout << "rob1inCam_mat4d " << endl << rob1inCam_mat4d << endl;

            // Handeye update ------------------
            //if (RunRobotIndex == 0)
            {
                Functions::Eigen2CVMat(rob0inCam_mat4d).copyTo(cHr0_updated);
                robotPosture robotpos_tmp = Functions::convertHomoMatrix2RobotPosture(rob0inCam_mat4d);
                for (int i=0; i<handeye_window; i++)
                {
                    robot0InCamera_hist[i].setPosition(robotpos_tmp.getPosition());
                    robot0InCamera_hist[i].setQuaternion(robotpos_tmp.getQuaternion());
                }
            }
            //if (RunRobotIndex == 1)
            {
                Functions::Eigen2CVMat(rob1inCam_mat4d).copyTo(cHr1_updated);
                robotPosture robotpos_tmp = Functions::convertHomoMatrix2RobotPosture(rob1inCam_mat4d);
                for (int i=0; i<handeye_window; i++)
                {
                    robot1InCamera_hist[i].setPosition(robotpos_tmp.getPosition());
                    robot1InCamera_hist[i].setQuaternion(robotpos_tmp.getQuaternion());
                }
            }
        }

            // Read tool in EE in reference trajectory -----------------------------
            ifstream f_stream_toolL(fname_eeHl), f_stream_toolR(fname_eeHr);
            static Matrix4d toolLInEE, toolRInEE;
            for (int i=0; i<4; i++)
                for (int j=0; j<4; j++)
                    {
                        double variable;
                        f_stream_toolL >> variable;
                        toolLInEE(i,j) = variable;
                        f_stream_toolR >> variable;
                        toolRInEE(i,j) = variable;
                    }

        //////////////////////////////////////////////

        // Draw hand eye result -------------------------------------------------
        Matrix4d robInCameraFrame, eeInRobFrame, markerInRobFrame;
        ros::spinOnce();
        if (RunRobotIndex == 0)
        {
            robInCameraFrame = rob0inCam_mat4d;
            eeInRobFrame = iiwa0_currentMartix4d;
            markerInRobFrame = eeInRobFrame * toolLInEE;
        }
        if (RunRobotIndex == 1)
        {
            robInCameraFrame = rob1inCam_mat4d;
            eeInRobFrame = iiwa1_currentMartix4d;
            markerInRobFrame = eeInRobFrame * toolRInEE;
        }
        Matrix4d markerInCamFrame_handeye = robInCameraFrame * markerInRobFrame;
        cv::Mat markerInHandEye =  Functions::Eigen2CVMat(markerInCamFrame_handeye);
        cout << "markerInHandEye " << markerInHandEye  << endl;
        myTracker->DrawHandEye = true;
        markerInHandEye.copyTo(myTracker->markerInHandEye_updated);

        ///////// Compute intial pose /////////
        cout<<"traj_ToolRInMan[0] ";
        traj_ToolRInMan[0].print();
        
        if (RunRobotIndex == 0 || RunRobotIndex == 2)
        {
            EEInRobot_0[0] = visionGuidedIIWAMoveToInitialPosture(traj_ToolLInMan[0], 0, rob0inCam_mat4d, toolLInEE, fname_eeHl);
            cout << "Computed EEInRobot_0: " << EEInRobot_0[0].getPosition().transpose() <<
                                        " " << EEInRobot_0[0].getQuaternion().w() <<
                                        " " << EEInRobot_0[0].getQuaternion().x() <<
                                        " " << EEInRobot_0[0].getQuaternion().y() <<
                                        " " << EEInRobot_0[0].getQuaternion().z() << endl;
        }
        if (RunRobotIndex == 1 || RunRobotIndex == 2)
        {
            EEInRobot_1[0] = visionGuidedIIWAMoveToInitialPosture(traj_ToolRInMan[0], 1, rob1inCam_mat4d, toolRInEE, fname_eeHr);

            cout << "Computed EEInRobot_1: " << EEInRobot_1[0].getPosition().transpose() <<
                                         " " << EEInRobot_1[0].getQuaternion().w() <<
                                         " " << EEInRobot_1[0].getQuaternion().x() <<
                                         " " << EEInRobot_1[0].getQuaternion().y() <<
                                         " " << EEInRobot_1[0].getQuaternion().z() << endl;
        }
        cout << "Robot 0 and 1 intitial poses computed!" << endl;
        if (RunRobotIndex == 0 || RunRobotIndex == 2)
        {
            status = 24;
            cout << "Status = " <<status << " !"<< endl;
            cout << "Robots 0 start moving to initial pose!" << endl;
        }
        else if (RunRobotIndex == 1 )
        {
            status = 25;
            cout << "Status = " <<status << " !"<< endl;
            cout << "Robots 1 start moving to initial pose!" << endl;
        }
        else
        {
            status = 100;
        }
    }

    else if (status == 24)
    {// Move robot 0 to initial poses

        /////////// Correct quaternion ///////////
        cout << "EEInRobot_0: " << EEInRobot_0[0].getPosition().transpose() << endl;

        // Move iiwa 0 to initial pose computed by hand-eye calibration
        Erl::Transformd iiwaTransformd = Functions::RobotPosture2ErlTransformd(EEInRobot_0[0]);

		ros::spinOnce();
        //cout << "iiwaCurrent" << endl << iiwas[0]->getiiwaPose() << endl;
        cout << "iiwaCurrent" << endl << iiwa0_currentTransformd << endl;
        cout << "iiwaTransformd" << endl << iiwaTransformd << endl;

		ros::spinOnce();
        bool iiwa_reached = moveIiwa(0, iiwaTransformd);//move iiwa0 and check if rechead
        //iiwas[0]->setiiwaPose(iiwaTransformd);
        //if (iiwas[0]->iiwaReached())
        if (iiwa_reached)
        {
						//cout << "iiwas[0]->iiwaReached() " << iiwas[0]->iiwaReached() << endl;
						cout << "iiwas[0]->iiwaReached() " << iiwa_reached << endl;
            cout << "Robot 0 finish moving to initial pose!" << endl;
            //cout << "iiwaFinished" << endl << iiwas[0]->getiiwaPose() << endl;
            cout << "iiwaFinished" << endl << iiwa0_currentTransformd << endl;
            myTracker->bool_recordTime = true;

            if (RunRobotIndex == 0 )
            {
                status = 26;
                recordRobotPose(traj_ToolLInMan, "traj_ToolLInMan_start.txt");
            }
            else if (RunRobotIndex == 2)
            {
                status = 25;
                cout << "Status = " <<status << " !"<< endl;
                cout << "Robots 1 start moving to initial pose!" << endl;
            }
            else
            {
                status = 100;
            }
            //status = 100;
            cout << "Status = " <<status << " !"<< endl;
        }
    }


    else if (status == 25)
    {// Move robot 1 to initial poses

        /////////// Correct quaternion ///////////
        cout << "EEInRobot_1: " << EEInRobot_1[0].getPosition().transpose() << endl;

        // Move iiwa 0 to initial pose computed by hand-eye calibration
        Erl::Transformd iiwaTransformd = Functions::RobotPosture2ErlTransformd(EEInRobot_1[0]);

		ros::spinOnce();
        //cout << "iiwaCurrent" << endl << iiwas[1]->getiiwaPose() << endl;
        cout << "iiwaCurrent" << endl << iiwa1_currentTransformd << endl;
        cout << "iiwaTransformd" << endl << iiwaTransformd << endl;


		ros::spinOnce();
        bool iiwa_reached = moveIiwa(1, iiwaTransformd);//move iiwa and check if reached
        //iiwas[1]->setiiwaPose(iiwaTransformd);
        //if (iiwas[1]->iiwaReached())
        if (iiwa_reached)
        {
            cout << "Robot 1 finish moving to initial pose!" << endl;
            //cout << "iiwaFinished" << endl << iiwas[1]->getiiwaPose() << endl;
            cout << "iiwaFinished at" << endl << iiwa1_currentTransformd << endl;
            myTracker->bool_recordTime = true;

            status = 26;//26
            cout << "Status = "<<status <<" !"<< endl;
            recordRobotPose(traj_ToolLInMan, "traj_ToolLInMan_start.txt");

        }

    }
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    /// //////////////////////////////////////////////////////////////////////////////
    ///
    else if (status == 26)
    {// For sewing: bimanual, execute both tools

        myTracker->PutText = true;

        ////////// Read all configs ///////////////
        // Read handeye ------------
        ifstream f_stream_r2c0(fname_cHr0), f_stream_r2c1(fname_cHr1);
        static Matrix4d rob0inCam_mat4d, rob1inCam_mat4d;
        for (int i=0; i<4; i++)
        {
            for (int j=0; j<4; j++)
                {
                    f_stream_r2c0 >> rob0inCam_mat4d(i,j);
                    f_stream_r2c1 >> rob1inCam_mat4d(i,j);
                }
        }

        // Read tool in EE-----------------------------
        ifstream f_stream_toolL(fname_eeHl), f_stream_toolR(fname_eeHr);
        static Matrix4d toolLInEE, toolRInEE;
        for (int i=0; i<4; i++)
            for (int j=0; j<4; j++)
                {
                    double variable;

                    f_stream_toolL >> variable;
                    toolLInEE(i,j) = variable;

                    f_stream_toolR >> variable;
                    toolRInEE(i,j) = variable;
                }
        //////////////////////////////////////////////


        //////////// Read marker in robot frame, and use handeye to transform to camera frame
#ifndef SimulationON

        Matrix4d toolLInRobFrame, markerLInCamFrame_handeye, markerLInCamFrame_handeye_updated;
        Matrix4d toolRInRobFrame, markerRInCamFrame_handeye, markerRInCamFrame_handeye_updated;

        // Draw hand eye result -------------------------------------------------
        Matrix4d robInCameraFrame, eeInRobFrame, markerInRobFrame;
        ros::spinOnce();
        if (RunRobotIndex == 0)
        {
            robInCameraFrame = rob0inCam_mat4d;
            eeInRobFrame = iiwa0_currentMartix4d;
            markerInRobFrame = eeInRobFrame * toolLInEE;
        }
        if (RunRobotIndex == 1)
        {
            robInCameraFrame = rob1inCam_mat4d;
            eeInRobFrame = iiwa1_currentMartix4d;
            markerInRobFrame = eeInRobFrame * toolRInEE;
        }
        Matrix4d markerInCamFrame_handeye = robInCameraFrame * markerInRobFrame;
        cv::Mat markerInHandEye =  Functions::Eigen2CVMat(markerInCamFrame_handeye);
        cout << "markerInHandEye " << markerInHandEye  << endl;
        myTracker->DrawHandEye = true;
        markerInHandEye.copyTo(myTracker->markerInHandEye_updated);



        if (RunRobotIndex == 0)
        {
            eeInRobFrame = iiwa0_currentMartix4d;
            markerInRobFrame = eeInRobFrame * toolLInEE;
            markerLInCamFrame_handeye_updated = Functions::CVMat2Eigen(cHr0_updated)* markerInRobFrame;
        }
        if (RunRobotIndex == 1)
        {
            eeInRobFrame = iiwa1_currentMartix4d;
            markerInRobFrame = eeInRobFrame * toolRInEE;
            markerRInCamFrame_handeye_updated = Functions::CVMat2Eigen(cHr1_updated)* markerInRobFrame;
        }

        cv::Mat markerLInHandEye_updated = Functions::Eigen2CVMat(markerLInCamFrame_handeye_updated);
        cv::Mat markerRInHandEye_updated = Functions::Eigen2CVMat(markerRInCamFrame_handeye_updated);

        // Only print tool R hand eye -------
        myTracker->DrawHandEye = true;
        if (RunRobotIndex == 0)
        {
            markerLInHandEye_updated.copyTo(myTracker->markerInHandEye_updated);
        }
        if (RunRobotIndex == 1)
        {
            markerRInHandEye_updated.copyTo(myTracker->markerInHandEye_updated);
        }

        // Record for hand eye -----------------------------------------------------
        aruco::Marker toolLtmp, toolRtmp;
        myTracker->toolsTracker->Tools[1].Tvec.copyTo(toolLtmp.Tvec);
        myTracker->toolsTracker->Tools[1].Rvec.copyTo(toolLtmp.Rvec);
        myTracker->toolsTracker->Tools[2].Tvec.copyTo(toolRtmp.Tvec);
        myTracker->toolsTracker->Tools[2].Rvec.copyTo(toolRtmp.Rvec);
#endif

//        //Kalman filter initialization ------------------------------
//        while (hist_EEInRobotBase.size()<CAMERALATENCY/ROBOTSLEEP)
//                hist_EEInRobotBase.push_back(Functions::Erl2RobotPosture(EE0Posture));

        ///////// Play trajectory /////////////////////////////////////////////////////////
        myTracker->Traj_index = trajIndex+1;

        if(RUNROBOT_mode == 1)
        {// Test mode:
            if (RunRobotIndex == 0 || RunRobotIndex == 2)
            {
                visionGuidedMoveToPosture_noVision_iiwa(traj_ToolLInMan[trajIndex_pre], traj_ToolLInMan[trajIndex], 0, rob0inCam_mat4d, toolLInEE, fname_eeHl, millisecbuffer, ms);
            }
            if (RunRobotIndex == 1 || RunRobotIndex == 2)
            {
                visionGuidedMoveToPosture_noVision_iiwa(traj_ToolRInMan[trajIndex_pre], traj_ToolRInMan[trajIndex], 1, rob1inCam_mat4d, toolRInEE, fname_eeHr, millisecbuffer, ms);
            }
        }
        else
        {
            cout<<"toolsDetected.size "<<myTracker->toolsTracker->toolsDetected.size()<<endl;
            cout<<"toolsDetected[0] "<<myTracker->toolsTracker->toolsDetected[0]<<endl;
            //while(myTracker->toolsTracker->toolsDetected.size()>0 &&
            //      !(myTracker->toolsTracker->toolsDetected[0]))
            ros::spinOnce();
        //        if (trajIndex%10==0)
            {
                if (RunRobotIndex == 0 || RunRobotIndex == 2)
                {
                    if (myTracker->toolsTracker->toolsDetected[1])
                    {
                        visionGuidedMoveToPosture_kalmanvision_iiwa(traj_ToolLInMan[trajIndex_pre], traj_ToolLInMan[trajIndex], 0, rob0inCam_mat4d, toolLInEE, fname_eeHl, millisecbuffer, ms);
                    }
                    else
                    {
                        visionGuidedMoveToPosture_noVision_iiwa(traj_ToolLInMan[trajIndex_pre], traj_ToolLInMan[trajIndex], 0, rob0inCam_mat4d, toolLInEE, fname_eeHl, millisecbuffer, ms);
                    }
                }
                if (RunRobotIndex == 1 || RunRobotIndex == 2)
                {
                    if (myTracker->toolsTracker->toolsDetected[2])
                    {
                        visionGuidedMoveToPosture_kalmanvision_iiwa(traj_ToolRInMan[trajIndex_pre], traj_ToolRInMan[trajIndex], 1, rob1inCam_mat4d, toolRInEE, fname_eeHr, millisecbuffer, ms);
                    }
                    else
                    {
                        visionGuidedMoveToPosture_noVision_iiwa(traj_ToolRInMan[trajIndex_pre], traj_ToolRInMan[trajIndex], 1, rob1inCam_mat4d, toolRInEE, fname_eeHr, millisecbuffer, ms);
                    }
                }
            }
        }

        if (initialise)
            initialise = false;

        cout << "trajIndex " << trajIndex << endl;
//        myNeedleDriver->changePos0( traj_DriverL[trajIndex]);
//        myNeedleDriver->changePos1(traj_DriverR[trajIndex]);

        trajIndex_pre = trajIndex;
        initTool_bool = false;
        trajIndex = trajIndex + TRAJSTEP;

        if (trajIndex > traj_ToolLInMan.size()-TRAJSTEP)
        {
            cout << "trajIndex " << trajIndex << " traj_ToolLInMan.size() " << traj_ToolLInMan.size()
                 << "TRAJSTEP " << TRAJSTEP << endl;
            status = 100;
//            msleep(1000);

            std_msgs::Bool iiwas_vs_reached_msg;
            iiwas_vs_reached_msg.data = true;
            pub_iiwas_vs_reached.publish(iiwas_vs_reached_msg);

            cout << "finish moving!" << endl;
        }
    }

    else if (status == 27) // Detect needle pose --------------------------
    {
        if (myTracker->needleDetected == false)
        {
            cout <<"Detecting needle!!!" << endl;
            myTracker->detectNeedle = true;
            myTracker->saveDetecedNeedle = true;
            status = 27;
        }
        else
        {
//            cout << "Status = 28!" << endl;
//            myTracker->detectNeedle = false;
//            msleep(2000);
//            status = 28;

            status = 40;
            myTracker->DrawNeedle = true;
            myTracker->detectNeedle = false;
            cout << "Status = " << status << endl;
            //msleep(2000);

        }
    }

    else if (status == 28)
    {
        myTracker->detectNeedle = false;

        Matrix4d lHn, lHn_;

        ifstream f_streamtHn(strdup((SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/tHn_.txt").c_str()));
          for (int i=0; i<4; i++)
              for (int j=0; j<4; j++)
                  {
                      double variable;
                      f_streamtHn >> variable;
                      lHn_(i,j)=variable;
                  }
          for (int i=0;i<4;i++)
              for (int j=0;j<4;j++)
              {
                  lHn(i,j)=iniToolLHNeedle_ei(i,j);
                  //lHn_(i,j)=myTracker->tHn_.at<double>(i,j);
              }

        cout << "lHn " << lHn  << endl;
        cout << "lHn_ " << lHn_ << endl;


        // Recompute 1L:

        // Clear previous trajectories -----------------------------------------------------
        myTracker->DrawTrajectory = false;
        EEInRobot_0.clear();
        traj_ToolLInMan.clear();
        traj_DriverL.clear();
        myTracker->toolLTraj_tvec.clear();

        // Re-read trajectories --------------------------------------------------------------

        // Robot frame and mandrel frame
        readToolsTrajinMandrelandDrivers(0, EEInRobot_0, traj_ToolLInMan, traj_DriverL,
                                         fname_cHr0, fname_eeHl, trajectFile);
        readToolsTrajinMandrelandDrivers(1, EEInRobot_1, traj_ToolRInMan, traj_DriverR,
                                         fname_cHr1, fname_eeHr, trajectFile);
        // Camera frame
        vector<robotPosture> traj_MandrelInCam, traj_ToolLInCam, traj_ToolRInCam;
        readMandrelToolsDrivers(trajectFile,traj_MandrelInCam,
                                traj_ToolLInCam,traj_ToolRInCam,
                                traj_DriverL, traj_DriverR);
        // 3D points in camera frame
        vector<cv::Point3f> man_traj, toolL_traj, toolR_traj, toolL_rvec, toolR_rvec;
        readMandrelToolsDriversInCamera(trajectFile, man_traj, toolL_traj, toolR_traj, toolL_rvec, toolR_rvec);
        for (int i=0; i<toolR_traj.size(); i++)
        {
            myTracker->toolLTraj_tvec.push_back(toolL_traj[i]);
            myTracker->toolRTraj_tvec.push_back(toolR_traj[i]);
            myTracker->toolLTraj_rvec.push_back(toolL_rvec[i]);
            myTracker->toolRTraj_rvec.push_back(toolR_rvec[i]);
        }

        // Re-compute trajectories ---------------------
        recordRobotPose(traj_ToolLInMan, "traj_ToolLInMan_init.txt");

        // Only re compute for tool L
        transferToolsTrajByNeedlePose(EEInRobot_0, traj_ToolLInMan, traj_ToolLInCam,
                                      myTracker->toolLTraj_tvec, myTracker->toolLTraj_rvec,
                                      Size_1L , lHn, lHn_,
                                      fname_cHr0, fname_eeHl, trajectFile);

        // Re-compute needle tip
        Mat nHp = Mat::eye(4,4,CV_64F);
        Mat tHp;
        int tipIndx = myTracker->NeedlePoints.size()-1;
        nHp.at<double>(0,3) = myTracker->NeedlePoints[tipIndx].x;
        nHp.at<double>(1,3) = myTracker->NeedlePoints[tipIndx].y;
        nHp.at<double>(2,3) = myTracker->NeedlePoints[tipIndx].z;
        tHp = Functions::Eigen2CVMat(lHn_) * nHp;
        tHp.copyTo(myTracker->ToolLHNeedleTip);

        cout << "tipIndx " << tipIndx << endl;
        cout << "nHp " << nHp << endl;
        cout << "tHp " << tHp << endl;
        cout <<"ToolLHNeedleTip " << myTracker->ToolLHNeedleTip << endl;


        myTracker->DrawTrajectory = true;
        recordRobotPose(traj_ToolLInMan, "traj_ToolLInMan_after.txt");


#ifdef SimulationON
#else
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    char buffer [80];
    strftime (buffer,80,"%Y-%m-%d-%H-%M",now);

    std::string toolsInCamFname, fEEInRobotsFname, fHandEye0Name,
                fToolTimerName, fEEInRobotsTimerFname;
    std::string fileDir=SRC_FILES_DIR+"demonstration/";
    toolsInCamFname = fileDir + "toolmandrel.repeat_" + string(buffer);
    fEEInRobotsFname = fileDir + "EEInRobots.repeat_" + string(buffer);
    fHandEye0Name = fileDir + "HandEye0_" + string(buffer);
    //fHandEye1Name = fileDir + "HandEye1_" + string(buffer);
    fToolTimerName = fileDir + "toolsTimer.txt_" + string(buffer);
    fEEInRobotsTimerFname = fileDir + "EEInRobotsTimer.repeat_" + string(buffer);

    toolsInCam.open(toolsInCamFname.c_str());
    fEEInRobots.open(fEEInRobotsFname.c_str());
    fHandEye0.open(fHandEye0Name.c_str());
//    fHandEye1.open(fHandEye1Name.c_str());
    fEEInRobots_timer.open(fEEInRobotsTimerFname.c_str());

    myTracker->initializeRecodeVideo(fileDir, buffer);
    myTracker->toolsInCamTimer.open(fToolTimerName.c_str());
#endif


        if (RunRobotIndex == 0)
        {
            status = 22;
            cout << "Status = 22!"<< endl;
            cout << "Robots 0 start moving to initial pose!" << endl;
        }
        else if (RunRobotIndex == 1 || RunRobotIndex == 2)
        {
            status = 21;
            cout << "Status = 21!" << endl;
            cout << "Robots 1 start moving to initial pose!" << endl;
        }
        else
        {
            status = 100;
        }

    }

    else if (status == 29) // Detect needle pose --------------------------
    {
        cout <<"Detecting needle!!!" << endl;
        myTracker->toolIndx_detectneedle = 2;
        myTracker->detectNeedle = true;
        myTracker->saveDetecedNeedle = false;
    }

    else if (status == 30)
    {
#ifdef iiwaOn
        Erl::Transformd rTee = iiwas[0]->getiiwaPose();
        Erl::Transformd rTee_ = rTee;
        Erl::Transformd eeTee_;

        double r = 30 * ERL_PI / 180;
//        r =0;
        Erl::Rotmatd eeRee_;
        eeRee_ = Erl::Rotmatd::fromEuler_XYZ(0.0,0.0,r);
        eeTee_.setRotation(eeRee_);
        eeTee_.setZ(-9);

        rTee_ = rTee * eeTee_;

        cout << "rTee " << rTee << endl;
        cout << "rTee_ " << rTee_ << endl;
        cout << "eeTee_ " << eeTee_ << endl;

        iiwas[0]->setiiwaPose(rTee_);
        iiwas[0]->iiwa.waitForDestinationReached();

        cout << "iiwa finished!" << endl;
      //  myTracker->DrawTrajectory = true;
#endif
        cout << "status == 100!" << endl;
        status = 100;
    }

    else if(status == 31)
    {// Show thread by force

        // Read force sensor ------------------------
//        int x, y, z;
//        x = forceReading->getX();
//        y = forceReading->getY();
//        z = forceReading->getZ();
//        cout << endl;
//        cout << "x: " << x << " y: " << y << " z: " << z << endl;

//        myTracker->DrawForce = true;
//        myTracker->forceX = -forceReading->getX();
//        myTracker->forceY = -forceReading->getY();
//        myTracker->forceZ = -forceReading->getZ();
    }

    else if (status == 40)
    {
        // Pause robot for human intervention ------------------

        // 1, Pause robot motion

        // 2, display detection result & Yes/No/Manual
        if (inspection_clicked != true)
        {
            emit checkImages(); // show image
        }
        else
        {
            inspection_clicked = false;
            if (inspection_answer == true)
            {
                status = 100;
                myTracker->DrawNeedle = false;
            }

            else
            {
                myTracker->DrawNeedle = false;
                status = 27; // detection is not good, detect again
            }
        }

        //



        //status = 100;
    }

    else if (status == 100)
    {
        //cout << "status == 100!" << endl;
    }


}




bool KukaMotionPlanning::readToolsTrajinMandrelandDrivers(int robotIndex,
                                                vector<robotPosture> &EEInRobot,
                                                vector<robotPosture> &traj_ToolInMan,
                                                vector<float> &traj_Driver,
                                                char* r2cFile, char* toolFile, char* trajectFile)
{
    EEInRobot.clear();
    traj_ToolInMan.clear();
    traj_Driver.clear();

    ////////// Read all configs ///////////////
    // Read handeye ------------
    ifstream r2c_stream(r2cFile);
    Matrix4d robInCameraFrame;
    for (int i=0; i<4; i++)
    {
        for (int j=0; j<4; j++)
            {
                r2c_stream >> robInCameraFrame(i,j);
            }
    }

    // Read tool in EE-----------------------------
    ifstream f_stream_tool(toolFile);
    Matrix4d toolInEE;
    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            {
                double variable;
                f_stream_tool >> variable;
                toolInEE(i,j) = variable;
            }

    ////////// Read learnt trajecotries ///////////////
    //Read learnt mandrel, tool left, tool right trajecotry ------------------------------
    std::vector<robotPosture> traj_MandrelInCam, traj_ToolLInCam, traj_ToolRInCam;
    vector<float> traj_DriverL, traj_DriverR;

//    readMandrelToolsPoses(trajectFile,
//                          traj_MandrelInCam,traj_ToolLInCam,traj_ToolRInCam);
    readMandrelToolsDrivers(trajectFile,traj_MandrelInCam,
                            traj_ToolLInCam,traj_ToolRInCam,
                            traj_DriverL, traj_DriverR);
    cout<< "finish reading trajectory in mandrel frame, points number: "<< traj_MandrelInCam.size()
        << " Robot index " << robotIndex <<endl;


    if (traj_MandrelInCam.size() > 0)
    {
        // Compute tool in mandrel
        if (robotIndex == 0)
        {
            computeToolInManTraj(traj_MandrelInCam,traj_ToolLInCam,traj_ToolInMan);
            for (int i=0; i<traj_DriverL.size(); i++)
            {
                traj_Driver.push_back(traj_DriverL[i]);
            }
        }
        else
        {
            computeToolInManTraj(traj_MandrelInCam,traj_ToolRInCam,traj_ToolInMan);
            for (int i=0; i<traj_DriverR.size(); i++)
            {
                traj_Driver.push_back(traj_DriverR[i]);
            }
        }

        cout<<"traj_ToolInMan from readToolsTrajinMandrelandDrivers"<<endl<<traj_ToolInMan[0].getPosition()<<" "<<traj_ToolInMan[0].getQuaternion().w()<<" "<<traj_ToolInMan[0].getQuaternion().x()<<" "<<traj_ToolInMan[0].getQuaternion().y()<<" "<<traj_ToolInMan[0].getQuaternion().z()<<" "<<endl;



        //////////// Detect tools ////////////////
        while(myTracker->toolsTracker->toolsDetected.size()>0 &&
              !(myTracker->toolsTracker->toolsDetected[0]))
        {
            cout << "Can't find mandrel!" << endl;
        }
        cout << "Found mandrel!" << endl;


        cv::Mat mandrelInCamFrame, toolInCamFrame;
        Matrix4d mandrelInRobFrame;
        // Mandrel ---------------
        myTracker->toolsTracker->toolPose2cvTrans(myTracker->toolsTracker->Tools[0], mandrelInCamFrame);
        mandrelInCamFrame.convertTo(mandrelInCamFrame, CV_64F);

        mandrelInCamFrame.copyTo(manInCam_ini);
        cout << "manInCam_ini " << manInCam_ini<< endl;


        // Tool  -----------------
        myTracker->toolsTracker->toolPose2cvTrans(myTracker->toolsTracker->Tools[robotIndex+1], toolInCamFrame);
        toolInCamFrame.convertTo(toolInCamFrame, CV_64F);

        //////////// Compute initial pose ////////////////
        // Compute robot trajectory in robot frame, according to learnt trajectory in mandrel frame --------------
        vector<robotPosture> traj_toolInRobot;

        mandrelInRobFrame = robInCameraFrame.inverse() * Functions::CVMat2Eigen(mandrelInCamFrame);
        tranTool2Robot(mandrelInRobFrame, traj_ToolInMan, traj_toolInRobot);
        tranToolInRob2NeeldeInRob(toolInEE, traj_toolInRobot, EEInRobot);
    ///////////////
        cout<<"------------------------------------------------"<<endl;
        cout<<"mandrelInRobFrame_org"<<endl<<mandrelInRobFrame<<endl;
    /////////////

    //////////
        cout<<"toolInCamFrame for robot " << robotIndex <<endl<<Functions::CVMat2Eigen(toolInCamFrame).inverse()<<endl;
        cout<<"toolInMandrelFrame for robot " << robotIndex << endl<<Functions::CVMat2Eigen(mandrelInCamFrame).inverse()* Functions::CVMat2Eigen(toolInCamFrame)<<endl;


     /////////////////
        Matrix4d toolInMan;
        toolInMan = Functions::CVMat2Eigen(mandrelInCamFrame).inverse() * Functions::CVMat2Eigen(toolInCamFrame);
    }
}



void KukaMotionPlanning::transferToolsTrajByNeedlePose(vector<robotPosture> &EEInRobot,
                                                       vector<robotPosture> &traj_ToolInMan,
                                                       vector<robotPosture> &traj_ToolInCamera,
                                                       vector<cv::Point3f> &tool_tvec, vector<cv::Point3f> &tool_rvec,
                                                       int size_seg,
                                                       Matrix4d tHn, Matrix4d tHn_,
                                                       char* r2cFile, char* toolFile, char* trajectFile
                                                       )
{   ////////// Read all configs ///////////////
    // Read handeye ------------
    ifstream r2c_stream(r2cFile);
    Matrix4d robInCameraFrame;
    for (int i=0; i<4; i++)
    {
        for (int j=0; j<4; j++)
            {
                r2c_stream >> robInCameraFrame(i,j);
            }
    }

    // Read tool in EE-----------------------------
    ifstream f_stream_tool(toolFile);
    Matrix4d toolInEE;
    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            {
                double variable;
                f_stream_tool >> variable;
                toolInEE(i,j) = variable;
            }

    ////////// Read learnt trajecotries ///////////////
//    //Read learnt mandrel, tool left, tool right trajecotry ------------------------------
//    std::vector<robotPosture> traj_MandrelInCam, traj_ToolLInCam, traj_ToolRInCam, traj_ToolInMan;
//    vector<float> traj_DriverL, traj_DriverR;
//    readMandrelToolsDrivers(trajectFile,traj_MandrelInCam,
//                            traj_ToolLInCam,traj_ToolRInCam,
//                            traj_DriverL, traj_DriverR);

//    // Compute tool in mandrel
//    if (robotIndex == 0)
//    {
//        computeToolInManTraj(traj_MandrelInCam,traj_ToolLInCam,traj_ToolInMan);
//    }
//    else
//    {
//        computeToolInManTraj(traj_MandrelInCam,traj_ToolRInCam,traj_ToolInMan);
//    }

    // Compute new tool in mandrel by new needle pose
    Matrix4d mHt_mat4d, cHt_mat4d;
    robotPosture robpose_temp;
    for (int i=0; i<size_seg; i++)
    {
        // Transform 1L in mandrel frame
        mHt_mat4d = Functions::convertRobotPosture2HomoMatrix(traj_ToolInMan[i]);
        mHt_mat4d = mHt_mat4d * tHn * tHn_.inverse();
        robpose_temp = Functions::convertHomoMatrix2RobotPosture(mHt_mat4d);
        traj_ToolInMan[i].setPosition(robpose_temp.getPosition());
        traj_ToolInMan[i].setQuaternion(robpose_temp.getQuaternion());

        // Transform 1L in camera frame for display
        cHt_mat4d = Functions::convertRobotPosture2HomoMatrix(traj_ToolInCamera[i]);
        cHt_mat4d = cHt_mat4d * tHn * tHn_.inverse();

        cv::Mat cHt = Functions::Eigen2CVMat(cHt_mat4d);
        cHt.convertTo(cHt, CV_32F);
        tool_tvec[i].x = cHt.at<float>(0,3);
        tool_tvec[i].y = cHt.at<float>(1,3);
        tool_tvec[i].z = cHt.at<float>(2,3);

        cv::Mat rvec;
        cv::Rodrigues(cHt.colRange(0,3).rowRange(0,3), rvec);
        tool_rvec[i].x = rvec.at<float>(0,0);
        tool_rvec[i].y = rvec.at<float>(1,0);
        tool_rvec[i].z = rvec.at<float>(2,0);
    }



    // Interpolate robot trajectory ---------------------------------------------------
    // TODO: tool_rvec ???????????????????????????????????????
    if (traj_ToolInMan.size() > size_seg)
    {
        int STEP = 100;
        Vector3d tempP0, tempP1;
        Quaterniond tempQ0, tempQ1;
        tempP0 = traj_ToolInMan[size_seg-1].getPosition();
        tempP1 = traj_ToolInMan[size_seg-1+STEP].getPosition();
        tempQ0 = traj_ToolInMan[size_seg-1].getQuaternion();
        tempQ1 = traj_ToolInMan[size_seg-1+STEP].getQuaternion();

        if (tempQ0.dot(tempQ1)<0)
        {// Check quaternion
            tempQ1.w() *= -1;
            tempQ1.x() *= -1;
            tempQ1.y() *= -1;
            tempQ1.z() *= -1;
            traj_ToolInMan[size_seg-1+STEP].setQuaternion(tempQ1);
        }

        for (int i=0; i<STEP; i++)
        {
            double para = double(i)/double(STEP);
            Vector3d interpP = Functions::linearInterpolation(tempP0, tempP1, para);
            Quaterniond interpQ = Functions::slerp(tempQ0, tempQ1, para);
            traj_ToolInMan[size_seg-1+i].setPosition(interpP);
            traj_ToolInMan[size_seg-1+i].setQuaternion(interpQ);
            cout << "para " << para << endl;
            cout << "interpP " << interpP << endl;
        }
    }

    cv::Mat mandrelInCamFrame, toolInCamFrame;
    Matrix4d mandrelInRobFrame;

    // Mandrel ---------------
    myTracker->toolsTracker->toolPose2cvTrans(myTracker->toolsTracker->Tools[0], mandrelInCamFrame);
    mandrelInCamFrame.convertTo(mandrelInCamFrame, CV_64F);
    mandrelInCamFrame.copyTo(manInCam_ini);
    // Tool  -----------------
//    myTracker->toolsTracker->toolPose2cvTrans(myTracker->toolsTracker->Tools[robotIndex+1], toolInCamFrame);
//    toolInCamFrame.convertTo(toolInCamFrame, CV_64F);


    // Compute robot trajectory in robot frame, according to learnt trajectory in mandrel frame --------------
    vector<robotPosture> traj_toolInRobot;
    mandrelInRobFrame = robInCameraFrame.inverse() * Functions::CVMat2Eigen(mandrelInCamFrame);
    tranTool2Robot(mandrelInRobFrame, traj_ToolInMan, traj_toolInRobot);

    EEInRobot.clear();
    tranToolInRob2NeeldeInRob(toolInEE, traj_toolInRobot, EEInRobot);


    // Debug ------------------------------------------------------------------------------
    record3DPose(tool_tvec, "tool_tvec_init.txt");

    for (int i=0; i<size_seg; i++)
    {
        // Transform 1L in camera frame for display
        Eigen::Matrix4d mHt_mat4d = Functions::convertRobotPosture2HomoMatrix(traj_ToolInMan[i]);
        cHt_mat4d = Functions::CVMat2Eigen(mandrelInCamFrame) * mHt_mat4d;

        cv::Mat cHt = Functions::Eigen2CVMat(cHt_mat4d);
        cHt.convertTo(cHt, CV_32F);
        tool_tvec[i].x = cHt.at<float>(0,3);
        tool_tvec[i].y = cHt.at<float>(1,3);
        tool_tvec[i].z = cHt.at<float>(2,3);

        cv::Mat rvec;
        cv::Rodrigues(cHt.colRange(0,3).rowRange(0,3), rvec);
        tool_rvec[i].x = rvec.at<float>(0,0);
        tool_rvec[i].y = rvec.at<float>(1,0);
        tool_rvec[i].z = rvec.at<float>(2,0);
    }
    record3DPose(tool_tvec, "tool_tvec_mHt.txt");
}






bool KukaMotionPlanning::visionGuidedMoveToPosture_kalmanvision_iiwa(robotPosture currentToolInMan,
                                                   robotPosture targetToolInMan, int robotIndex,
                                                   Matrix4d robInCameraFrame,
                                                   Matrix4d toolInEE, char *toolFile,
                                                   char *millisecbuffer, int ms)
{
    cv::Mat mandrelInCamFrame, toolInCamFrame;
    Matrix4d toolInRobFrame, desiredToolInRob, deltaToolInRob, desiredEEInRob;
    robotPosture EEInRob_robpose;
    float speedlimit;

    ///////////////// Read current mandrel and tool poses /////////////////
    aruco::Marker man, tool;
    myTracker->toolsTracker->Tools[0].Tvec.copyTo(man.Tvec);
    myTracker->toolsTracker->Tools[0].Rvec.copyTo(man.Rvec);
    man.id = myTracker->toolsTracker->Tools[0].id;
    myTracker->toolsTracker->Tools[robotIndex+1].Tvec.copyTo(tool.Tvec);
    myTracker->toolsTracker->Tools[robotIndex+1].Rvec.copyTo(tool.Rvec);
    tool.id = myTracker->toolsTracker->Tools[robotIndex+1].id;

    myTracker->toolsTracker->toolPose2cvTrans(man, mandrelInCamFrame);
    mandrelInCamFrame.convertTo(mandrelInCamFrame, CV_64F);

    // ======== HARDCODE TO FIXED POSITION =======================
    manInCam_ini.copyTo(mandrelInCamFrame);
    // ===========================================================

    myTracker->toolsTracker->toolPose2cvTrans(tool, toolInCamFrame);
    toolInCamFrame.convertTo(toolInCamFrame, CV_64F);

    // Read current tool position in robot
    toolInRobFrame = readToolPoseInIIWAbase(robotIndex, toolFile);

    // Record current position detected by camera ------------------------------
    for(int k=0; k<3; k++)
    {
        toolsInCam << myTracker->toolsTracker->Tools[k].Tvec.ptr<float>(0)[0] << " "
                   << myTracker->toolsTracker->Tools[k].Tvec.ptr<float>(0)[1] << " "
                   << myTracker->toolsTracker->Tools[k].Tvec.ptr<float>(0)[2] << " "
                   << myTracker->toolsTracker->Tools[k].Rvec.ptr<float>(0)[0] << " "
                   << myTracker->toolsTracker->Tools[k].Rvec.ptr<float>(0)[1] << " "
                   << myTracker->toolsTracker->Tools[k].Rvec.ptr<float>(0)[2] << " ";
    }

    toolsInCam << myTracker->toolsTracker->Tools[0].id << " "
               << myTracker->toolsTracker->Tools[1].id << " "
               << myTracker->toolsTracker->Tools[2].id << " ";


    /////////////// Compute next desired EE pose in robot /////////////////

#ifdef SimulationON
    speedlimit = 0.5;
    desiredToolInRob = robInCameraFrame.inverse() * Functions::CVMat2Eigen(mandrelInCamFrame) *
                       Functions::convertRobotPosture2HomoMatrix(targetToolInMan);
    desiredToolInRob = deltaToolInRob * toolInRobFrame;
    cout << "deltaToolInRob " << robotIndex << " kalman simulation: " <<endl;
#else
    speedlimit = 0.05;
    Marker tmpMarker;

    if (mandrelInCamFrame.at<double>(0,3) != -1000 && toolInCamFrame.at<double>(0,3) != -1000
            && tool.Tvec.ptr<float>(0)[0] != -1000)
    {
        cout << "Before computer filter " << endl;
        /// *************kalmanfilter******************************
        ///
        ///
        ///
        /// -------------------------------------------------------
        ///
        ///
        cv::Mat x_k(6,1, CV_64F), z_k(6,1, CV_64F), x_k_kal(6,1, CV_64F);
        cv::Mat x_k_rot, z_k_rot;
        cv::Mat x_k_mat, z_k_mat, x_k_kal_mat;
        Matrix4d toolInRobFrame_preL, x_k_mat4d, z_k_mat4d;
        Matrix3d x_k_rotmat3d, z_k_rotmat3d;
        Quaterniond x_k_quat, z_k_quat, x_k_kal_quat;
        //Marker tmpMarker;
        Vector3d x_k_euler3d, z_k_euler3d;
        cv::Vec3d x_k_euler, z_k_euler;

        int index = cnt_robothist + 1;
        if (index == hist_EEInRobotBase.size())
            index = 0;
        //toolInRobFrame_preL = Functions::convertRobotPosture2HomoMatrix(hist_EEInRobotBase[index]) * toolInEE;

            x_k_mat4d = Functions::CVMat2Eigen(mandrelInCamFrame) *
                        Functions::convertRobotPosture2HomoMatrix(currentToolInMan);
//        x_k_mat4d = robInCameraFrame * toolInRobFrame;
//        z_k_mat4d = Functions::CVMat2Eigen(toolInCamFrame)*
//                      toolInRobFrame_preL.inverse() *
//                      toolInRobFrame;
        z_k_mat4d = Functions::CVMat2Eigen(toolInCamFrame);

        x_k_quat = Functions::Rot2Quaternion(x_k_mat4d);
        z_k_quat = Functions::Rot2Quaternion(z_k_mat4d);

        x_k_mat = Functions::Eigen2CVMat(x_k_mat4d);
        z_k_mat = Functions::Eigen2CVMat(z_k_mat4d);

        x_k_rot = x_k_mat.colRange(0,3).rowRange(0,3);
        z_k_rot = z_k_mat.colRange(0,3).rowRange(0,3);

//        cout << "x_k_mat " << x_k_mat << endl;
//        cout << "z_k_mat " << z_k_mat << endl;
//        cout << "toolInCamFrame " << toolInCamFrame << endl;
//        cout << "toolInRobFrame_preL " << toolInRobFrame_preL << endl;
//        cout << "toolInRobFrame " << toolInRobFrame << endl;

        myTracker->toolsTracker->cvTrans2toolPose(x_k_mat, tmpMarker);

        x_k.at<double>(0,0) = tmpMarker.Tvec.ptr<float>(0)[0];
        x_k.at<double>(1,0) = tmpMarker.Tvec.ptr<float>(0)[1];
        x_k.at<double>(2,0) = tmpMarker.Tvec.ptr<float>(0)[2];
        x_k.at<double>(3,0) = tmpMarker.Rvec.ptr<float>(0)[0];
        x_k.at<double>(4,0) = tmpMarker.Rvec.ptr<float>(0)[1];
        x_k.at<double>(5,0) = tmpMarker.Rvec.ptr<float>(0)[2];

        myTracker->toolsTracker->cvTrans2toolPose(z_k_mat, tmpMarker);
        z_k.at<double>(0,0) = tmpMarker.Tvec.ptr<float>(0)[0];
        z_k.at<double>(1,0) = tmpMarker.Tvec.ptr<float>(0)[1];
        z_k.at<double>(2,0) = tmpMarker.Tvec.ptr<float>(0)[2];
        z_k.at<double>(3,0) = tmpMarker.Rvec.ptr<float>(0)[0];
        z_k.at<double>(4,0) = tmpMarker.Rvec.ptr<float>(0)[1];
        z_k.at<double>(5,0) = tmpMarker.Rvec.ptr<float>(0)[2];
//        cout << "x_k_rot " << x_k_rot << endl;
//        cout << "z_k_rot " << z_k_rot << endl;
        x_k_euler = Functions::rotationMatrixToEulerAngles(x_k_rot);
        z_k_euler = Functions::rotationMatrixToEulerAngles(z_k_rot);

        for (int a=0; a<3; a++)
        {
            if (x_k_euler[a] - z_k_euler[a] > 5)
            {
                z_k_euler[a] = z_k_euler[a] + 2 * M_PI  ;
            }
            if (x_k_euler[a] - z_k_euler[a] < -5)
            {
                x_k_euler[a] = x_k_euler[a] + 2 * M_PI  ;
            }
        }
//        cout << "x_k_euler " << x_k_euler << endl;
//        cout << "z_k_euler " << z_k_euler << endl;
        x_k.at<double>(3,0) = x_k_euler[0];
        x_k.at<double>(4,0) = x_k_euler[1];
        x_k.at<double>(5,0) = x_k_euler[2];
        z_k.at<double>(3,0) = z_k_euler[0];
        z_k.at<double>(4,0) = z_k_euler[1];
        z_k.at<double>(5,0) = z_k_euler[2];
//        cout << "x_k " << x_k << endl;
//        cout << "z_k " << z_k << endl;
        P_kalman = P_kalman + Q_kalman;
        P_kalman = P_kalman - K_kalman*P_kalman;
        K_kalman = P_kalman/(P_kalman+R_kalman);

        x_k_kal = x_k + K_kalman *(z_k - x_k);

////         Test Only Compensate for latency--------------
////        x_k_kal = z_k;
//        cout << "K " << K_kalman << endl;
//        cout << "P " << P_kalman << endl;
//        cout << "x_k_kal " << x_k_kal << endl;
        tmpMarker.Tvec.ptr<float>(0)[0] = x_k_kal.at<double>(0,0);
        tmpMarker.Tvec.ptr<float>(0)[1] = x_k_kal.at<double>(1,0);
        tmpMarker.Tvec.ptr<float>(0)[2] = x_k_kal.at<double>(2,0);
        tmpMarker.Rvec.ptr<float>(0)[0] = x_k_kal.at<double>(3,0);
        tmpMarker.Rvec.ptr<float>(0)[1] = x_k_kal.at<double>(4,0);
        tmpMarker.Rvec.ptr<float>(0)[2] = x_k_kal.at<double>(5,0);

        myTracker->toolsTracker->toolPose2cvTrans(tmpMarker, x_k_kal_mat);

    //    // Test Quaternion --------------------------
    //    x_k_kal_quat = Functions::slerp(z_k_quat, x_k_quat, K_kalman.at<double>(3,3));
    //    Matrix3d tmpRot = x_k_kal_quat.toRotationMatrix();
    //    Functions::Eigen2CVMat(tmpRot).copyTo(x_k_kal_mat.colRange(0,3).rowRange(0,3));
    //    // -------------------------------

        // Test Euler angles ----------------------------
        x_k_euler[0] = x_k_kal.at<double>(3,0);
        x_k_euler[1] = x_k_kal.at<double>(4,0);
        x_k_euler[2] = x_k_kal.at<double>(5,0);
        Functions::eulerAnglesToRotationMatrix(x_k_euler).copyTo(x_k_kal_mat.colRange(0,3).rowRange(0,3));

        x_k_kal_mat.convertTo(x_k_kal_mat,CV_64F);
        x_k_kal_mat.copyTo(myTracker->markerInHandEye);
        cout << "x_k_kal_mat: " << x_k_kal_mat << endl;
        cout << "toolInCamFrame " << toolInCamFrame << endl;
        /// ******************************************************************************
        ///
        ///
        ros::spinOnce();
        robInCameraFrame = Functions::CVMat2Eigen(x_k_kal_mat) * toolInRobFrame.inverse();
        desiredToolInRob = robInCameraFrame.inverse() * Functions::CVMat2Eigen(mandrelInCamFrame) *
                           Functions::convertRobotPosture2HomoMatrix(targetToolInMan);
        cout << "deltaToolInRob " << robotIndex << " kalman vision: " << endl;

        UpdateHandEye(Functions::convertHomoMatrix2RobotPosture(robInCameraFrame), robotIndex);
    }
    else
    {
        if (robotIndex == 0)
        {
            robInCameraFrame = Functions::CVMat2Eigen(cHr0_updated);
        }
        if (robotIndex == 1)
        {
            robInCameraFrame = Functions::CVMat2Eigen(cHr1_updated);
        }

        desiredToolInRob = robInCameraFrame.inverse() * Functions::CVMat2Eigen(mandrelInCamFrame) *
                           Functions::convertRobotPosture2HomoMatrix(targetToolInMan);

        cout << "deltaToolInRob " << robotIndex << " no detect kalman: " << endl;
    }
#endif

    desiredEEInRob = desiredToolInRob * toolInEE.inverse();
    Matrix4d deltaT = toolInRobFrame.inverse() * desiredToolInRob;
    cout << deltaT << endl;
    cout << "toolInRobFrame " << endl << toolInRobFrame << endl;
    cout << "robInCameraFrame " << endl << robInCameraFrame << endl;
    cout << "mandrelInCamFrame " << endl << mandrelInCamFrame << endl;
    cout << "targetToolInMan " << endl << Functions::convertRobotPosture2HomoMatrix(targetToolInMan) << endl;


    // Move iiwa to initial pose ---------------
    Erl::Transformd iiwaTransformd = Functions::Eigen2Erl(desiredEEInRob);
    //fEEInRobots << iiwaTransformd << endl;
    robotPosture EEInRobCurrPose;
    if (robotIndex == 0){
        EEInRobCurrPose = Functions::convertHomoMatrix2RobotPosture(iiwa0_currentMartix4d);
    }
    if (robotIndex == 1){
        EEInRobCurrPose = Functions::convertHomoMatrix2RobotPosture(iiwa1_currentMartix4d);
    }


    fEEInRobots << " " << EEInRobCurrPose.getPosition().transpose()
                << " " << EEInRobCurrPose.getQuaternion().w()
                << " " << EEInRobCurrPose.getQuaternion().x()
                << " " << EEInRobCurrPose.getQuaternion().y()
                << " " << EEInRobCurrPose.getQuaternion().z()<<endl;

    //cout << "iiwaCurrent" << endl << iiwas[robotIndex]->getiiwaPose() << endl;
    ros::spinOnce();
    if (robotIndex == 0)
    {
    	cout << "iiwa0Current" << endl << iiwa0_currentTransformd << endl;
    }else if (robotIndex == 1)
    {
    	cout << "iiwa1Current" << endl << iiwa1_currentTransformd << endl;
    }
    
    cout << "iiwaTransformd" << endl << iiwaTransformd << endl;
    cout << "matrix 4d" << endl << desiredEEInRob<<endl;

    //move iiwa
    moveIiwa(robotIndex,iiwaTransformd);

	


//*********** Move robot *******************************
#ifdef iiwaOn
    iiwas[robotIndex]->setiiwaPose(iiwaTransformd);
    while(!iiwas[robotIndex]->iiwaReached())
    {
    }
#endif
// *****************************************************

#ifdef SimulationON
#else
       // Save command ------------------------------------
       //Matrix4d eeInRobByCam = robInCameraFrame.inverse() * Functions::CVMat2Eigen(toolLInCamFrame) * toolLInEE.inverse();
/*       Matrix4d eeInRobDesired_mat4d = robInCameraFrame.inverse() * Functions::CVMat2Eigen(mandrelInCamFrame) *
                                       Functions::convertRobotPosture2HomoMatrix(targetToolInMan)
                                       * toolInEE.inverse();
       robotPosture eeInRobDesired = Functions::convertHomoMatrix2RobotPosture(eeInRobDesired_mat4d);

       // Update robot current pose -----------------------------

       fEEInRobots << EEInRobCurrPose.getPosition().transpose()
                   << " " << EEInRobCurrPose.getQuaternion().w()
                   << " " << EEInRobCurrPose.getQuaternion().x()
                   << " " << EEInRobCurrPose.getQuaternion().y()
                   << " " << EEInRobCurrPose.getQuaternion().z()
                   << " ";
       fEEInRobots << EEInRob_robpose.getPosition().transpose()
                   << " " << EEInRob_robpose.getQuaternion().w()
                   << " " << EEInRob_robpose.getQuaternion().x()
                   << " " << EEInRob_robpose.getQuaternion().y()
                   << " " << EEInRob_robpose.getQuaternion().z()
                   << " " << millisecbuffer << ms
                   << " ";

       EEInRobCurr= dualArmPlanner->getRobotInstance(robotIndex)->getHomoMatrix();
       EEInRobCurrPose = Functions::convertHomoMatrix2RobotPosture(EEInRobCurr);

       fEEInRobots << EEInRobCurrPose.getPosition().transpose()
                   << " " << EEInRobCurrPose.getQuaternion().w()
                   << " " << EEInRobCurrPose.getQuaternion().x()
                   << " " << EEInRobCurrPose.getQuaternion().y()
                   << " " << EEInRobCurrPose.getQuaternion().z()
                   << endl;
*/
       // Dectect marker after moving ------------------------------------
       for(int k=0; k<3; k++)
       {
           toolsInCam << myTracker->toolsTracker->Tools[k].Tvec.ptr<float>(0)[0] << " "
                      << myTracker->toolsTracker->Tools[k].Tvec.ptr<float>(0)[1] << " "
                      << myTracker->toolsTracker->Tools[k].Tvec.ptr<float>(0)[2] << " "
                      << myTracker->toolsTracker->Tools[k].Rvec.ptr<float>(0)[0] << " "
                      << myTracker->toolsTracker->Tools[k].Rvec.ptr<float>(0)[1] << " "
                      << myTracker->toolsTracker->Tools[k].Rvec.ptr<float>(0)[2] << " ";
       }
       toolsInCam << millisecbuffer << ms << " ";

       toolsInCam << tmpMarker.Tvec.ptr<float>(0)[0] << " "
                  << tmpMarker.Tvec.ptr<float>(0)[1] << " "
                  << tmpMarker.Tvec.ptr<float>(0)[2] << " "
                  << tmpMarker.Rvec.ptr<float>(0)[0] << " "
                  << tmpMarker.Rvec.ptr<float>(0)[1] << " "
                  << tmpMarker.Rvec.ptr<float>(0)[2] << " "
                  << endl;
       //-------------------------------------------------

			// Get Kuka current Joints
/*			Eigen::Matrix<double, 7, 1> currJoints = iiwas[robotIndex]->iiwa.getJoints();
			fKukaJointsTraj << currJoints[0] << " " << currJoints[1] << " " << currJoints[2] << " "
											<< currJoints[3] << " " << currJoints[4] << " " << currJoints[5] << " "
											<< currJoints[6] <<endl;
*/			
#endif

}



bool KukaMotionPlanning::visionGuidedMoveToPosture_noVision_iiwa(robotPosture initToolInMan,
                                                                   robotPosture targetToolInMan, int robotIndex,
                                                                   Matrix4d robInCameraFrame,
                                                                   Matrix4d toolInEE, char *toolFile,
                                                                   char *millisecbuffer, int ms)
{
    cv::Mat mandrelInCamFrame, toolInCamFrame;
    Matrix4d toolInRobFrame, desiredToolInRob, deltaToolInRob, desiredEEInRob;
    robotPosture EEInRob_robpose;
    float speedlimit;


    ///////////////// Read current mandrel and tool poses /////////////////
    aruco::Marker man, tool;
    myTracker->toolsTracker->Tools[0].Tvec.copyTo(man.Tvec);
    myTracker->toolsTracker->Tools[0].Rvec.copyTo(man.Rvec);
    man.id = myTracker->toolsTracker->Tools[0].id;
    myTracker->toolsTracker->Tools[robotIndex+1].Tvec.copyTo(tool.Tvec);
    myTracker->toolsTracker->Tools[robotIndex+1].Rvec.copyTo(tool.Rvec);
    tool.id = myTracker->toolsTracker->Tools[robotIndex+1].id;

    //myTracker->toolsTracker->toolPose2cvTrans(myTracker->toolsTracker->Tools[0], mandrelInCamFrame);
    myTracker->toolsTracker->toolPose2cvTrans(man, mandrelInCamFrame);
    mandrelInCamFrame.convertTo(mandrelInCamFrame, CV_64F);

    // HARDCODE TO FIXED POSITION ----------------------
    manInCam_ini.copyTo(mandrelInCamFrame);

    myTracker->toolsTracker->toolPose2cvTrans(tool, toolInCamFrame);
    toolInCamFrame.convertTo(toolInCamFrame, CV_64F);

        for(int k=0; k<3; k++)
        {
            toolsInCam << myTracker->toolsTracker->Tools[k].Tvec.ptr<float>(0)[0] << " "
                       << myTracker->toolsTracker->Tools[k].Tvec.ptr<float>(0)[1] << " "
                       << myTracker->toolsTracker->Tools[k].Tvec.ptr<float>(0)[2] << " "
                       << myTracker->toolsTracker->Tools[k].Rvec.ptr<float>(0)[0] << " "
                       << myTracker->toolsTracker->Tools[k].Rvec.ptr<float>(0)[1] << " "
                       << myTracker->toolsTracker->Tools[k].Rvec.ptr<float>(0)[2] << " ";
        }

        toolsInCam << myTracker->toolsTracker->Tools[0].id << " "
                   << myTracker->toolsTracker->Tools[1].id << " "
                   << myTracker->toolsTracker->Tools[2].id << " ";


    // Read current tool position in robot
        if (initTool_bool == false)
        {
            initToolInRobFrame = readToolPoseInIIWAbase(robotIndex, toolFile);
            initTool_bool = true;
        }
        toolInRobFrame = readToolPoseInIIWAbase(robotIndex, toolFile);

    /////////////// Compute next desired EE pose in robot /////////////////

#ifdef SimulationON
    speedlimit = 0.5;
    desiredToolInRob = robInCameraFrame.inverse() * Functions::CVMat2Eigen(mandrelInCamFrame) *
                       Functions::convertRobotPosture2HomoMatrix(targetToolInMan);
//    cout << "deltaTool " << robotIndex << " no vision simulation: " <<endl;
#else
    speedlimit = 0.1;


    if (robotIndex == 0)
    {
        robInCameraFrame = Functions::CVMat2Eigen(cHr0_updated);
    }
    if (robotIndex == 1)
    {
        robInCameraFrame = Functions::CVMat2Eigen(cHr1_updated);
    }


    if (mandrelInCamFrame.at<double>(0,3) != -1000 && toolInCamFrame.at<double>(0,3) != -1000
            && tool.Tvec.ptr<float>(0)[0] != -1000)
    {
        desiredToolInRob = robInCameraFrame.inverse() * Functions::CVMat2Eigen(mandrelInCamFrame) *
                           Functions::convertRobotPosture2HomoMatrix(targetToolInMan);
        cout << "deltaEEInRob " << robotIndex << " no vision: " << endl;
    }
    else
    {
        desiredToolInRob = robInCameraFrame.inverse() * Functions::CVMat2Eigen(mandrelInCamFrame) *
                           Functions::convertRobotPosture2HomoMatrix(targetToolInMan);
        cout << "deltaEEInRob " << robotIndex << " no detect: " << endl;
    }
#endif

    desiredEEInRob = desiredToolInRob * toolInEE.inverse();
    Matrix4d deltaT = toolInRobFrame.inverse() * desiredToolInRob;

    // Need this????
    while (abs(deltaT(0,3)) > speedlimit || abs(deltaT(1,3)) > speedlimit||abs(deltaT(2,3)) > speedlimit)
    {
        cout << "No vision Reach velocity maximum " << speedlimit << " : " <<deltaT(0,3) << " " << deltaT(1,3) <<
                " " << deltaT(2,3) << " Stop!" << endl;
        msleep(500);
    }

    Erl::Transformd iiwaTransformd = Functions::Eigen2Erl(desiredEEInRob);

	ros::spinOnce();
	if (robotIndex == 0)
	{
		cout << "iiwaCurrent" << endl << iiwa0_currentTransformd << endl;
	}else if (robotIndex == 1)
	{
		cout << "iiwaCurrent" << endl << iiwa1_currentTransformd << endl;
	}
    //cout << "iiwaCurrent" << endl << iiwas[robotIndex]->getiiwaPose() << endl;
    cout << "iiwaTransformd" << endl << iiwaTransformd << endl;

//    myTracker->DrawHandEye = true;

	
	//move iiwa
    moveIiwa(robotIndex,iiwaTransformd);

#ifdef SimulationON
#else
//    iiwas[robotIndex]->setiiwaPose(iiwaTransformd);
//   while(!iiwas[robotIndex]->iiwaReached())
//    {
//    }
#endif


       // Save command ------------------------------------
       Matrix4d eeInRobDesired_mat4d = robInCameraFrame.inverse() * Functions::CVMat2Eigen(mandrelInCamFrame) *
                                       Functions::convertRobotPosture2HomoMatrix(targetToolInMan)
                                       * toolInEE.inverse();
       robotPosture eeInRobDesired = Functions::convertHomoMatrix2RobotPosture(eeInRobDesired_mat4d);

       //Erl::Transformd erl = iiwas[robotIndex]->getiiwaPose();
       ros::spinOnce();
       Erl::Transformd erl;
       if (robotIndex == 0)
       {
       		erl = iiwa0_currentTransformd;
       }else if (robotIndex == 1)
       {
       		erl = iiwa1_currentTransformd;
       }
       Matrix4d EEInRobCurr = Functions::Erl2Eigen(erl);
       robotPosture EEInRobCurrPose = Functions::convertHomoMatrix2RobotPosture(EEInRobCurr);
       fEEInRobots << " " << EEInRobCurrPose.getPosition().transpose()
                   << " " << EEInRobCurrPose.getQuaternion().w()
                   << " " << EEInRobCurrPose.getQuaternion().x()
                   << " " << EEInRobCurrPose.getQuaternion().y()
                   << " " << EEInRobCurrPose.getQuaternion().z();
       if (robotIndex == 1)
            fEEInRobots << endl;
       //--------------------------------------------------

       // Dectect marker after moving ------------------------------------
       for(int k=0; k<3; k++)
       {
           toolsInCam << myTracker->toolsTracker->Tools[k].Tvec.ptr<float>(0)[0] << " "
                      << myTracker->toolsTracker->Tools[k].Tvec.ptr<float>(0)[1] << " "
                      << myTracker->toolsTracker->Tools[k].Tvec.ptr<float>(0)[2] << " "
                      << myTracker->toolsTracker->Tools[k].Rvec.ptr<float>(0)[0] << " "
                      << myTracker->toolsTracker->Tools[k].Rvec.ptr<float>(0)[1] << " "
                      << myTracker->toolsTracker->Tools[k].Rvec.ptr<float>(0)[2] << " ";
       }
       toolsInCam << millisecbuffer << ms << " ";

       toolsInCam << -1000 << " " << -1000 << " " << -1000 << " "
                  << -1000 << " " << -1000 << " " << -1000 << endl;

       //--------------------------------------------------
			// Get Kuka current Joints
/*			Eigen::Matrix<double, 7, 1> currJoints = iiwas[robotIndex]->iiwa.getJoints();
			fKukaJointsTraj << currJoints[0] << " " << currJoints[1] << " " << currJoints[2] << " "
											<< currJoints[3] << " " << currJoints[4] << " " << currJoints[5] << " "
											<< currJoints[6] <<endl;
*/
}



robotPosture KukaMotionPlanning::visionGuidedIIWAMoveToInitialPosture(
                                   robotPosture targetToolInMan, int robotIndex,
                                   Matrix4d robInCameraFrame,
                                   Matrix4d toolInEE, char *toolFile)
{
    cv::Mat mandrelInCamFrame, toolInCamFrame;
    Matrix4d toolInRobFrame, desiredToolInRob, deltaToolInRob, desiredEEInRob;
    robotPosture EEInRob_robpose;

    ///////////////// Read current mandrel and tool poses /////////////////
    aruco::Marker man, tool;
    man.Tvec.ptr<float>(0)[0] = -1000;
    tool.Tvec.ptr<float>(0)[0] = -1000;

    int temp_iter = 0;
    while(man.Tvec.ptr<float>(0)[0] == -1000 ||
            tool.Tvec.ptr<float>(0)[0] == -1000)
    {
        myTracker->toolsTracker->Tools[0].Tvec.copyTo(man.Tvec);
        myTracker->toolsTracker->Tools[0].Rvec.copyTo(man.Rvec);
        myTracker->toolsTracker->Tools[robotIndex+1].Tvec.copyTo(tool.Tvec);
        myTracker->toolsTracker->Tools[robotIndex+1].Rvec.copyTo(tool.Rvec);

#ifndef SimulationON
        temp_iter ++;
        if (temp_iter > 1000)
        {
            cout << "Have not found tools " << robotIndex << " ! Manually added tool pose!" << endl;
            manInCam_ini.copyTo(mandrelInCamFrame);
            Matrix4d mHt = Functions::convertRobotPosture2HomoMatrix(targetToolInMan);
            cv::Mat toolInCam_tmp = manInCam_ini * Functions::Eigen2CVMat(mHt);
            myTracker->toolsTracker->cvTrans2toolPose(toolInCam_tmp, tool);
            break;
        }
#else
        cout << "Have not found tools " << robotIndex << " !" << endl;
#endif
    }
    cout << "Tool found!" << endl;

    myTracker->toolsTracker->toolPose2cvTrans(man, mandrelInCamFrame);
    mandrelInCamFrame.convertTo(mandrelInCamFrame, CV_64F);

    // HARDCODE TO FIXED POSITION ----------------------
    manInCam_ini.copyTo(mandrelInCamFrame);

    myTracker->toolsTracker->toolPose2cvTrans(tool, toolInCamFrame);
    toolInCamFrame.convertTo(toolInCamFrame, CV_64F);

    // Read current tool position in robot
    toolInRobFrame = readToolPoseInIIWAbase(robotIndex, toolFile);


    /////////////// Compute next desired EE pose in robot /////////////////
//    deltaToolInRob = robInCameraFrame.inverse() * Functions::CVMat2Eigen(mandrelInCamFrame) *
//                        Functions::convertRobotPosture2HomoMatrix(targetToolInMan)
//                        * (robInCameraFrame.inverse() * Functions::CVMat2Eigen(toolInCamFrame)).inverse();

//    desiredToolInRob = deltaToolInRob * toolInRobFrame;

    desiredToolInRob = robInCameraFrame.inverse() * Functions::CVMat2Eigen(mandrelInCamFrame) *
                       Functions::convertRobotPosture2HomoMatrix(targetToolInMan);
    cout<<"visionGuidedIIWAMoveToInitialPosture robInCameraFrame"<<endl<<robInCameraFrame<<endl;
    cout<<"visionGuidedIIWAMoveToInitialPosture mandrelInCamFrame"<<endl<<mandrelInCamFrame<<endl;
    cout<<"visionGuidedIIWAMoveToInitialPosture targetToolInMan"<<endl<<targetToolInMan<<endl;

    desiredEEInRob = desiredToolInRob * toolInEE.inverse();

    EEInRob_robpose = Functions::convertHomoMatrix2RobotPosture(desiredEEInRob);
    cout << "visionGuidedIIWAMoveToInitialPosture EEInRob_robpose " ;
    EEInRob_robpose.print();

    return EEInRob_robpose;
}






//////////////////////////
void KukaMotionPlanning::readMandrelToolsPoses( char * fileDir, std::vector<robotPosture> &mandrel , std::vector<robotPosture> &toolL, vector<robotPosture> &toolR)
{
    //std::vector<robotPosture> needleDriver0Pose;
    std::ifstream  fneedleDriversPoses(fileDir, std::ios::in);
    std::string  px0, py0, pz0, rx0, ry0, rz0, px1, py1, pz1, rx1, ry1, rz1, px2, py2, pz2, rx2, ry2, rz2;

    char  line[1024]={0};
    while(fneedleDriversPoses.getline(line, sizeof(line)))
    {
        std::stringstream  word(line);
        word >> px0; word >> py0; word >> pz0;
        word >> rx0; word >> ry0; word >> rz0;
        word >> px1; word >> py1; word >> pz1;
        word >> rx1; word >> ry1; word >> rz1;
        word >> px2; word >> py2; word >> pz2;
        word >> rx2; word >> ry2; word >> rz2;

        robotPosture poseTemp;
        cv::Mat Rvec(3,1,CV_64F);
        cv::Mat rotCV;
        Eigen::Matrix3d rotEigen;
        Quaterniond quat;

        //Mandrel ------------
        poseTemp.setPosition(   atof(px0.c_str()),   atof(py0.c_str()),   atof(pz0.c_str())  );

        Rvec.at<double>(0,0) = atof(rx0.c_str());
        Rvec.at<double>(1,0) = atof(ry0.c_str());
        Rvec.at<double>(2,0) = atof(rz0.c_str());
        cv::Rodrigues(Rvec, rotCV);
        rotEigen = Eigen::Matrix3d(Functions::CVMat2Eigen(rotCV));
        quat = Functions::Rot2Quaternion(rotEigen);
        poseTemp.setQuaternion(quat);

        mandrel.push_back(poseTemp);

        //ToolL ------------
        poseTemp.setPosition(   atof(px1.c_str()),   atof(py1.c_str()),   atof(pz1.c_str())  );

        Rvec.at<double>(0,0) = atof(rx1.c_str());
        Rvec.at<double>(1,0) = atof(ry1.c_str());
        Rvec.at<double>(2,0) = atof(rz1.c_str());

        cv::Rodrigues(Rvec, rotCV);
        rotEigen = Eigen::Matrix3d(Functions::CVMat2Eigen(rotCV));
        //quat=rotEigen;
        quat = Functions::Rot2Quaternion(rotEigen);
        poseTemp.setQuaternion(quat);

        toolL.push_back(poseTemp);

        //ToolR ------------
        poseTemp.setPosition(   atof(px2.c_str()),   atof(py2.c_str()),   atof(pz2.c_str())  );

        Rvec.at<double>(0,0) = atof(rx2.c_str());
        Rvec.at<double>(1,0) = atof(ry2.c_str());
        Rvec.at<double>(2,0) = atof(rz2.c_str());

        cv::Rodrigues(Rvec, rotCV);
        rotEigen = Eigen::Matrix3d(Functions::CVMat2Eigen(rotCV));
        //quat=rotEigen;
        quat = Functions::Rot2Quaternion(rotEigen);
        poseTemp.setQuaternion(quat);

        toolR.push_back(poseTemp);

//        cout << "Rvec: " << Rvec << endl;
//        cout << "rotCV: " << rotCV << endl;
//        cout << "rotEigen: " << rotEigen << endl;

//        cout << "quat: " << quat.w() << " "
//                         << quat.x() << " "
//                         << quat.y() << " "
//                         << quat.z() << endl;
    }

    fneedleDriversPoses.close();
   //return needleDriver0Pose;
}

void KukaMotionPlanning::
readMandrelToolsDrivers( char * fileDir, vector<robotPosture> &mandrel,
                          vector<robotPosture> &toolL, vector<robotPosture> &toolR,
                          vector<float> &driverL, vector<float> &driverR)
{
    mandrel.clear(); toolL.clear(); toolR.clear(); driverL.clear(); driverR.clear();

    //std::vector<robotPosture> needleDriver0Pose;
    std::ifstream  fneedleDriversPoses(fileDir, std::ios::in);

    std::string  px0, py0, pz0, rx0, ry0, rz0,
                 px1, py1, pz1, rx1, ry1, rz1,
                 px2, py2, pz2, rx2, ry2, rz2,
                 dr0, dr1;

    char  line[1024]={0};
    while(fneedleDriversPoses.getline(line, sizeof(line)))
    {
        std::stringstream  word(line);
        word >> px0; word >> py0; word >> pz0;
        word >> rx0; word >> ry0; word >> rz0;
        word >> px1; word >> py1; word >> pz1;
        word >> rx1; word >> ry1; word >> rz1;
        word >> px2; word >> py2; word >> pz2;
        word >> rx2; word >> ry2; word >> rz2;
        word >> dr0; word >> dr1;

        robotPosture poseTemp;
        cv::Mat Rvec(3,1,CV_64F);
        cv::Mat rotCV;
        Eigen::Matrix3d rotEigen;
        Quaterniond quat;

        //Mandrel ------------
        poseTemp.setPosition(   atof(px0.c_str()),   atof(py0.c_str()),   atof(pz0.c_str())  );

        Rvec.at<double>(0,0) = atof(rx0.c_str());
        Rvec.at<double>(1,0) = atof(ry0.c_str());
        Rvec.at<double>(2,0) = atof(rz0.c_str());
        cv::Rodrigues(Rvec, rotCV);
        rotEigen = Eigen::Matrix3d(Functions::CVMat2Eigen(rotCV));
        quat = Functions::Rot2Quaternion(rotEigen);
        poseTemp.setQuaternion(quat);

        mandrel.push_back(poseTemp);

        //ToolL ------------
        poseTemp.setPosition(   atof(px1.c_str()),   atof(py1.c_str()),   atof(pz1.c_str())  );

        Rvec.at<double>(0,0) = atof(rx1.c_str());
        Rvec.at<double>(1,0) = atof(ry1.c_str());
        Rvec.at<double>(2,0) = atof(rz1.c_str());

        cv::Rodrigues(Rvec, rotCV);
        rotEigen = Eigen::Matrix3d(Functions::CVMat2Eigen(rotCV));
        //quat=rotEigen;
        quat = Functions::Rot2Quaternion(rotEigen);
        poseTemp.setQuaternion(quat);

        toolL.push_back(poseTemp);

        //ToolR ------------
        poseTemp.setPosition(   atof(px2.c_str()),   atof(py2.c_str()),   atof(pz2.c_str())  );

        Rvec.at<double>(0,0) = atof(rx2.c_str());
        Rvec.at<double>(1,0) = atof(ry2.c_str());
        Rvec.at<double>(2,0) = atof(rz2.c_str());

        cv::Rodrigues(Rvec, rotCV);
        rotEigen = Eigen::Matrix3d(Functions::CVMat2Eigen(rotCV));
        //quat=rotEigen;
        quat = Functions::Rot2Quaternion(rotEigen);
        poseTemp.setQuaternion(quat);

        toolR.push_back(poseTemp);

        // Needle drivers -----------------
        int drL, drR;
        drL = atof(dr0.c_str());
        drR = atof(dr1.c_str());
        if (drL>=0 && drL<=4500 && drR>=0 && drR<=4500)
        {
            driverL.push_back(drL);
            driverR.push_back(drR);
            //cout << "driverL " << drL << " driverR " << drR << endl;
        }

//        cout << "Rvec: " << Rvec << endl;
//        cout << "rotCV: " << rotCV << endl;
//        cout << "rotEigen: " << rotEigen << endl;

//        cout << "quat: " << quat.w() << " "
//                         << quat.x() << " "
//                         << quat.y() << " "
//                         << quat.z() << endl;
    }

    fneedleDriversPoses.close();
   //return needleDriver0Pose;
}


void KukaMotionPlanning::readEETraj(char * fileDir, vector<robotPosture> &EE)
{
    // This fuction reads the EE trajectory in robot base frame
    // Input file format: 4 *4 homo matrix
    EE.clear();
    std::ifstream  fstream_EE(fileDir);
    char  line[1024]={0};
    string x, y, z, w, a, b, c;

    while(fstream_EE.getline(line, sizeof(line)))
    {
        robotPosture tmp_pose;
        std::stringstream  word(line);
        word>>w; word>>a; word>>b; word>>c;
        word>>x; word>>y; word>>z;
        tmp_pose.setPosition((double)atof(x.c_str()), (double)atof(y.c_str()), (double)atof(z.c_str()));
        tmp_pose.setQuaternion((double)atof(w.c_str()), (double)atof(a.c_str()), (double)atof(b.c_str()), (double)atof(c.c_str()));

        EE.push_back(tmp_pose);
    }
    fstream_EE.close();
}


void KukaMotionPlanning::readMandrelToolsDriversInCamera( char * fileDir, vector<cv::Point3f> &mandrel,
                                                  vector<cv::Point3f> &toolL_tvec, vector<cv::Point3f> &toolR_tvec,
                                                  vector<cv::Point3f> &toolL_rvec, vector<cv::Point3f> &toolR_rvec)
{
    // 1. Read trajectories in camera frame
    std::vector<robotPosture> traj_MandrelInCam, traj_ToolLInCam, traj_ToolRInCam;
    std::vector<robotPosture> traj_ToolLInMan, traj_ToolRInMan;
    vector<float> traj_DriverL, traj_DriverR;

    readMandrelToolsDrivers(fileDir,traj_MandrelInCam,
                            traj_ToolLInCam,traj_ToolRInCam,
                            traj_DriverL, traj_DriverR);

    // 2. Compute trajectories w.r.t mandrel frame
    computeToolInManTraj(traj_MandrelInCam,traj_ToolLInCam,traj_ToolLInMan);
    computeToolInManTraj(traj_MandrelInCam,traj_ToolRInCam,traj_ToolRInMan);

    // 3. Compute back the trajectories in camera frame
    Matrix4d mHl, mHr, cHl, cHr, cHm;
    cHm  = Functions::CVMat2Eigen(manInCam_ini);
    Mat rotCV;
    Mat Rvec(3,1,CV_32F);

    for (int i=0; i<traj_MandrelInCam.size(); i++)
    {
        // 3.1 convert trajectories robotPosture -> matrix

        mHl = Functions::convertRobotPosture2HomoMatrix(traj_ToolLInMan[i]);
        mHr = Functions::convertRobotPosture2HomoMatrix(traj_ToolRInMan[i]);

        // 3.2 multipled by current mandrel pose in camera frame

        cHl = cHm * mHl;
        cHr = cHm * mHr;

        // 3.3 convert matrix to tvec, rvec
        mandrel.push_back(Point3f(cHm(0,3), cHm(1,3), cHm(2,3)));
        toolL_tvec.push_back(Point3f(cHl(0,3), cHl(1,3), cHl(2,3)));
        toolR_tvec.push_back(Point3f(cHr(0,3), cHr(1,3), cHr(2,3)));
				/////////////////////
				//toolL_tvec.push_back(Point3f(traj_ToolLInCam[i].getPosition()[0], traj_ToolLInCam[i].getPosition()[1], traj_ToolLInCam[i].getPosition()[2]));
				//cout<<"toolL_tvec "<<cHl(0,3)<<" "<<cHl(1,3)<<" "<<cHl(2,3)<<endl;
				//cout<<"traj_ToolLInCam "<<traj_ToolLInCam[i].getPosition()[0]<<endl;
                //cHr_traj<<cHr(0,3)<<" "<<cHr(1,3)<<" "<<cHr(2,3)<<endl;
				///////////////////////
        rotCV = Functions::Eigen2CVMat(cHl);
        cv::Rodrigues(rotCV.colRange(0,3).rowRange(0,3), Rvec);
        toolL_rvec.push_back(Point3f(Rvec));

        rotCV = Functions::Eigen2CVMat(cHr);
        cv::Rodrigues(rotCV.colRange(0,3).rowRange(0,3), Rvec);
        toolR_rvec.push_back(Point3f(Rvec));
    }


}

/////////////////////////////////////////////////////////////////////

void KukaMotionPlanning::computeToolInManTraj(vector<robotPosture> &traj_MandrelInCam,
                                              vector<robotPosture> &traj_ToolLInCam,
                                              vector<robotPosture> &traj_ToolLInMan)
{
    // Inputs: traj_MandrelInCam, traj_ToolLInCam. Outputs: traj_ToolLInMan -----


    for (int i=0; i<traj_ToolLInCam.size(); i++)
    {
        Matrix4d manInCam_trans, toolInCam_trans, toolInMan_trans;
        manInCam_trans = Functions::convertRobotPosture2HomoMatrix(traj_MandrelInCam[i]);
        toolInCam_trans = Functions::convertRobotPosture2HomoMatrix(traj_ToolLInCam[i]);

        toolInMan_trans = manInCam_trans.inverse() * toolInCam_trans;

        robotPosture toolInManTemp = Functions::convertHomoMatrix2RobotPosture( toolInMan_trans );
        traj_ToolLInMan.push_back(toolInManTemp);
        
        if(i == 0){
        	cout<<"manInCam_trans"<<endl<<manInCam_trans<<endl;
        	cout<<"toolInCam_trans"<<endl<<toolInCam_trans<<endl;
        	cout<<"toolInMan_trans"<<endl<<toolInMan_trans<<endl;
            cout<<"traj_ToolLInMan"<<endl<<traj_ToolLInMan[0].getPosition()<<" "<<traj_ToolLInMan[0].getQuaternion().w()<<" "<<traj_ToolLInMan[0].getQuaternion().x()<<" "<<traj_ToolLInMan[0].getQuaternion().y()<<" "<<traj_ToolLInMan[0].getQuaternion().z()<<endl;
        	}
    }
    
    
}


void KukaMotionPlanning::tranTool2Robot(Matrix4d manInRob_trans, vector<robotPosture> &toolInMan, vector<robotPosture> &toolInRob)
{
    // Inputs: manInRob_trans, toolInMan. Outputs: toolInRob -----

    for (int i=0; i<toolInMan.size(); i++)
    {
        Matrix4d toolInRob_trans;
        Matrix4d toolInMan_trans = Functions::convertRobotPosture2HomoMatrix(toolInMan[i]);

        toolInRob_trans = manInRob_trans * toolInMan_trans;

//        cout << "toolInMan_trans: " << toolInMan_trans << endl;
//        cout << "toolInRob_trans: " << toolInRob_trans << endl;
        robotPosture toolInRobTemp = Functions::convertHomoMatrix2RobotPosture( toolInRob_trans );
        toolInRob.push_back(toolInRobTemp);
    }
}

void KukaMotionPlanning::tranToolInRob2NeeldeInRob(Matrix4d toolInEE_trans, vector<robotPosture> &toolInRob, vector<robotPosture> &EEInRob)
{
    // Inputs: toolInEE_trans, toolInRob. Outputs: EEInRob -----
    Matrix4d EEInRob_trans;
    for (int i=0; i<toolInRob.size(); i++)
    {
        Matrix4d toolInRob_trans = Functions::convertRobotPosture2HomoMatrix(toolInRob[i]);

        EEInRob_trans = toolInRob_trans * toolInEE_trans.inverse();

        robotPosture EEInRobTemp = Functions::convertHomoMatrix2RobotPosture( EEInRob_trans );
        EEInRob.push_back(EEInRobTemp);
    }
//    cout<<"toolInEE_trans"<<endl<<toolInEE_trans<<endl;
//    cout<<"toolInRob"<<endl<<toolInRob[0]<<endl;
//    cout<<"EEInRob"<<endl<<EEInRob[0]<<endl;
}


Matrix4d KukaMotionPlanning::readToolPoseInIIWAbase(int robotRef, char* toolFile)
{
    ifstream f_stream3(toolFile);

    // Read from iiwa ------------------------------------
    //Erl::Transformd rTee = iiwas[robotRef]->getiiwaPose();
	ros::spinOnce();
	Erl::Transformd rTee;
	if (robotRef == 0)
	{
		rTee = iiwa0_currentTransformd;
	}else if (robotRef == 1)
	{
		rTee = iiwa1_currentTransformd;
	}
    cout<<"rTee"<<endl<<rTee<<endl;

    Eigen::Matrix4d EE_eigen = Functions::Erl2Eigen(rTee);
    Matrix4d markerInTip;
    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            {
                double variable;
                f_stream3 >> variable;
                markerInTip(i,j) =variable;
            }

    Matrix4d marker2Base=EE_eigen*markerInTip;
    return marker2Base;
}


//Matrix4d KukaMotionPlanning::readToolPoseInBaseFrame(int robotRef, char* toolFile)
//{
//    Matrix4d TipInBaseRob0= dualArmPlanner->getRobotInstance(robotRef)->getHomoMatrix();
//    ifstream f_stream3(toolFile);
//    Matrix4d markerInTip;
//    for (int i=0; i<4; i++)
//        for (int j=0; j<4; j++)
//            {
//                double variable;
//                f_stream3 >> variable;
//                markerInTip(i,j) =variable;
//            }
//    Matrix4d marker2Base=TipInBaseRob0*markerInTip;
//    return marker2Base;
//}


void KukaMotionPlanning::setOpMode(int opMode)
{
    controlopMode=opMode;

}

void KukaMotionPlanning::onAndOffButton(bool button)
{
    onAndOff=button;
}

void KukaMotionPlanning::plannerStop()
{

}

void KukaMotionPlanning::plannerStart()
{

}

void KukaMotionPlanning::enableGravy()
{
    cout << "applyGravy" << endl;
//    iiwas[0]->/.();
}
void KukaMotionPlanning::disableGravy()
{
    cout << "disGravy" << endl;
/*    if (iiwas[0]->iiwaConnected == true)
        iiwas[0]->DisableGravityCompensation();
    if (iiwas[1]->iiwaConnected == true)
        iiwas[1]->DisableGravityCompensation();*/
}

void KukaMotionPlanning::plannerClose()
{
}

void KukaMotionPlanning::samplingKukaStares()
{

}


void KukaMotionPlanning::inspection_yes_clicked()
{
//    cout<< "inspection_yes_clicked!"<< endl;
//    inspection_answer = true;
//    inspection_clicked = true;
//    Inspection->hide();
//    DispThread->hide();
}

void KukaMotionPlanning::inspection_no_clicked()
{
//    cout<< "inspection_no_clicked!"<< endl;
//    inspection_answer = false;
//    inspection_clicked = true;
//    Inspection->hide();
//    DispThread->hide();
}


void KukaMotionPlanning::UpdateHandEye(robotPosture robotInCam, int robotIndx)
{
    double x, y, z, a, b, c, w;
    x = 0; y = 0; z = 0; a = 0; b = 0; c = 0; w = 0;

    if (robotIndx == 0)
    {
        robot0InCamera_hist[handeye_count0].setPosition(robotInCam.getPosition());
        robot0InCamera_hist[handeye_count0].setQuaternion(robotInCam.getQuaternion());
        for(int i=0; i<handeye_window; i++)
        {
            x = x + robot0InCamera_hist[i].getPosition()(0);
            y = y + robot0InCamera_hist[i].getPosition()(1);
            z = z + robot0InCamera_hist[i].getPosition()(2);
            w = w + robot0InCamera_hist[i].getQuaternion().w();
            a = a + robot0InCamera_hist[i].getQuaternion().x();
            b = b + robot0InCamera_hist[i].getQuaternion().y();
            c = c + robot0InCamera_hist[i].getQuaternion().z();
        }
    }
    else
    {
        robot1InCamera_hist[handeye_count1].setPosition(robotInCam.getPosition());
        robot1InCamera_hist[handeye_count1].setQuaternion(robotInCam.getQuaternion());
        for(int i=0; i<handeye_window; i++)
        {
            x = x + robot1InCamera_hist[i].getPosition()(0);
            y = y + robot1InCamera_hist[i].getPosition()(1);
            z = z + robot1InCamera_hist[i].getPosition()(2);
            w = w + robot1InCamera_hist[i].getQuaternion().w();
            a = a + robot1InCamera_hist[i].getQuaternion().x();
            b = b + robot1InCamera_hist[i].getQuaternion().y();
            c = c + robot1InCamera_hist[i].getQuaternion().z();
        }
    }



    robotPosture robPos_tmp;
    robPos_tmp.setPosition(x/handeye_window, y/handeye_window, z/handeye_window);
    robPos_tmp.setQuaternion(w/handeye_window, a/handeye_window,
                             b/handeye_window, c/handeye_window);
/*
    robPos_tmp.setPosition(robotInCam.getPosition());
    robPos_tmp.setQuaternion(robotInCam.getQuaternion());*/



    Matrix4d cHr_tmp = Functions::convertRobotPosture2HomoMatrix(robPos_tmp);

    if (robotIndx == 0)
    {
        Functions::Eigen2CVMat(cHr_tmp).copyTo(cHr0_updated);
        handeye_count0++;
        if (handeye_count0 == handeye_window)
            handeye_count0 = 0;

        fHandEye0 << robPos_tmp.getPosition()(0) << " "
                  << robPos_tmp.getPosition()(1) << " "
                  << robPos_tmp.getPosition()(2) << " "
                  << robPos_tmp.getQuaternion().w() << " " << robPos_tmp.getQuaternion().x() << " "
                  << robPos_tmp.getQuaternion().y() << " " << robPos_tmp.getQuaternion().z() << endl;
    }
    if (robotIndx == 1)
    {
        Functions::Eigen2CVMat(cHr_tmp).copyTo(cHr1_updated);
        handeye_count1++;
        if (handeye_count1 == handeye_window)
            handeye_count1 = 0;

        fHandEye1 << robPos_tmp.getPosition()(0) << " "
                  << robPos_tmp.getPosition()(1) << " "
                  << robPos_tmp.getPosition()(2) << " "
                  << robPos_tmp.getQuaternion().w() << " " << robPos_tmp.getQuaternion().x() << " "
                  << robPos_tmp.getQuaternion().y() << " " << robPos_tmp.getQuaternion().z() << endl;
    }


}

void KukaMotionPlanning::recordRobotPose(vector<robotPosture> &EEInRobot, char *fname)
{
    ofstream f_robpose;
    f_robpose.open(fname);

    for (int i=0; i<EEInRobot.size(); i++)
    {
        f_robpose << EEInRobot[i].getPosition().transpose() << " "
                  << EEInRobot[i].getQuaternion().w() << " "
                  << EEInRobot[i].getQuaternion().x() << " "
                  << EEInRobot[i].getQuaternion().y() << " "
                  << EEInRobot[i].getQuaternion().z()
                  << endl;
    }

    f_robpose.close();
}

void KukaMotionPlanning::record3DPose(vector<cv::Point3f> &Tvec, char *fname)
{
    ofstream f_robpose;
    f_robpose.open(fname);

    for (int i=0; i<Tvec.size(); i++)
    {
        f_robpose << Tvec[i].x << " "
                  << Tvec[i].y << " "
                  << Tvec[i].z << endl;
    }

    f_robpose.close();
}




void KukaMotionPlanning::computeThreadFrame(Mat thread_, Point3d tool_, Mat &frame_)
{
    Point3d o, t, l, ol, X, Y, Z;
    o.x = thread_.at<double>(0,0);    o.y = thread_.at<double>(0,1);    o.z = thread_.at<double>(0,2);
    t.x = thread_.at<double>(1,0);    t.y = thread_.at<double>(1,1);    t.z = thread_.at<double>(1,2);
    l.x = tool_.x;    l.y = tool_.y;    l.z = tool_.z;

    ol = l - o;
    X = t - o;
    Y = ol.cross(X);
    Z = X.cross(Y);

    double x_norm, y_norm, z_norm;
    x_norm = sqrt(X.x*X.x + X.y*X.y + X.z*X.z);
    y_norm = sqrt(Y.x*Y.x + Y.y*Y.y + Y.z*Y.z);
    z_norm = sqrt(Z.x*Z.x + Z.y*Z.y + Z.z*Z.z);

    X.x = X.x / x_norm; X.y = X.y / x_norm; X.z = X.z / x_norm;
    Y.x = Y.x / y_norm; Y.y = Y.y / y_norm; Y.z = Y.z / y_norm;
    Z.x = Z.x / z_norm; Z.y = Z.y / z_norm; Z.z = Z.z / z_norm;

    frame_.at<double>(0,0)=X.x; frame_.at<double>(0,1)=Y.x; frame_.at<double>(0,2)=Z.x;
    frame_.at<double>(1,0)=X.y; frame_.at<double>(1,1)=Y.y; frame_.at<double>(1,2)=Z.y;
    frame_.at<double>(2,0)=X.z; frame_.at<double>(2,1)=Y.z; frame_.at<double>(2,2)=Z.z;

    frame_.at<double>(0,3)=o.x;
    frame_.at<double>(1,3)=o.y;
    frame_.at<double>(2,3)=o.z;
}

Mat KukaMotionPlanning::threadTransform(Mat thread_old, Mat thread_new, Point3d tool_old, Point3d tool_new)
{

    //Mat4f homo;
    cv::Mat homo(4,4,CV_64F);

    Mat H_old = cv::Mat::eye(4,4,CV_64F);
    Mat H_new = cv::Mat::eye(4,4,CV_64F);
    computeThreadFrame(thread_old, tool_old, H_old);
    computeThreadFrame(thread_new, tool_new, H_new);

//    // Draw thread frame ------------------------------------------
//    Marker tmp;
//    tmp.Tvec.ptr<float>(0)[0] = (float)H_new.at<double>(0,3);
//    tmp.Tvec.ptr<float>(0)[1] = (float)H_new.at<double>(1,3);
//    tmp.Tvec.ptr<float>(0)[2] = (float)H_new.at<double>(2,3);

//    cv::Rodrigues(H_new.colRange(0,3).rowRange(0,3), tmp.Rvec);

//    tmp.ssize = 0.01;

//    CvDrawingUtils::draw3dAxis(dst_l, tmp, toolsTracker->StereoCameraParametersL);
//    imwrite("left_frame.png", dst_l);
//    // ============================================================

    cout << "H_old " << H_old << endl;
    cout << "H_new " << H_new << endl;
    homo = H_new * H_old.inv();
    cout << "homo " << homo << endl;
    return homo;
}

void KukaMotionPlanning::adaptToolRbyThread(vector<robotPosture> &traj_ToolRInCam)
{
    ifstream fstream_thread_ori(myTracker->fname_thread_ori);
    ifstream fstream_thread_new(myTracker->fname_thread_new);

     Mat thread_ori = Mat(2,3, CV_64F, double(0));
     Mat thread_new = Mat(2,3, CV_64F, double(0));
     for (int i=0; i<2; i++)
         for (int j=0; j<3; j++)
             {
                 double variable_ori, variable_new;
                 fstream_thread_ori >> variable_ori;
                 thread_ori.at<double>(i,j) = variable_ori;

//                 thread_new.at<double>(i,j) = variable_ori;

                 fstream_thread_new >> variable_new;
                 thread_new.at<double>(i,j) = variable_new;
             }

     Point3d toolR_o, toolR_n;
     toolR_o.x = traj_ToolRInCam[0].getPosition()[0];
     toolR_o.y = traj_ToolRInCam[0].getPosition()[1];
     toolR_o.z = traj_ToolRInCam[0].getPosition()[2];

     toolR_n.x = myTracker->toolsTracker->Tools[2].Tvec.ptr<float>(0)[0];
     toolR_n.y = myTracker->toolsTracker->Tools[2].Tvec.ptr<float>(0)[1];
     toolR_n.z = myTracker->toolsTracker->Tools[2].Tvec.ptr<float>(0)[2];

    Mat deltaThread = threadTransform(thread_ori, thread_new, toolR_o, toolR_n);

    // Transform toolR trajectory
    Matrix4d homo_traj;
   for (int i=0; i < traj_ToolRInCam.size(); i++)
   {
       homo_traj = Functions::convertRobotPosture2HomoMatrix(traj_ToolRInCam[i]);
       homo_traj = Functions::CVMat2Eigen(deltaThread) * homo_traj;
       traj_ToolRInCam[i] = Functions::convertHomoMatrix2RobotPosture(homo_traj);
   }

   fstream_thread_ori.close();
   fstream_thread_new.close();
}





void KukaMotionPlanning::checkQuaternion(Eigen::Quaterniond &quad_curr, Eigen::Quaterniond &quad_next)
{
    if (quad_curr.dot(quad_next) < 0)
    {
        quad_next.w() *= -1;
        quad_next.x() *= -1;
        quad_next.y() *= -1;
        quad_next.z() *= -1;

        cout << "Quaternion reverse!" << endl;
    }
}

void KukaMotionPlanning::recordforLiang()
{
    myTracker->SaveImage = true;
    cout << "recordforLiang " << endl;
}

bool KukaMotionPlanning::moveIiwa(int robotRef, Erl::Transformd iiwa_transformd){

    ros::Rate rate(rate_hz);

    if (!iiwa0_transformd_received){
        tmp_iiwa0_transformd = iiwa_transformd;
        iiwa0_transformd_received = true;
        return true;
    }

    if (iiwa0_transformd_received){
        iiwa0_transformd_received = false;
        Erl::Transformd tmp_iiwa1_transformd = iiwa_transformd;

        int64_t nowms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        iiwa0_transformd_file<<nowms<<" ";
        iiwa1_transformd_file<<nowms<<" ";

        //record kuka current transformd
        for (int i = 0; i <3; i++){
            for(int j = 0; j <4; j ++){
                iiwa0_transformd_file<<iiwa0_currentTransformd(i,j)<<" ";
                iiwa1_transformd_file<<iiwa1_currentTransformd(i,j)<<" ";

            }
        }

        //record kuka next transformd
        for (int i = 0; i <3; i++){
            for(int j = 0; j <4; j ++){
                iiwa0_transformd_file<<tmp_iiwa0_transformd(i,j)<<" ";
                iiwa1_transformd_file<<tmp_iiwa1_transformd(i,j)<<" ";
            }
        }

        iiwa0_transformd_file<<endl;
        iiwa1_transformd_file<<endl;

        if (exotica_complete){
            std_msgs::Float64MultiArray msg_iiwa0_transformd, msg_iiwa1_transformd;
            for (int i = 0; i <3;i++){
                for (int j = 0; j<4; j++){
                    if (j == 3){
                        msg_iiwa0_transformd.data.push_back(tmp_iiwa0_transformd(i,j)/1000.0);
                        msg_iiwa1_transformd.data.push_back(tmp_iiwa1_transformd(i,j)/1000.0);
                    }else{
                        msg_iiwa0_transformd.data.push_back(tmp_iiwa0_transformd(i,j));
                        msg_iiwa1_transformd.data.push_back(tmp_iiwa1_transformd(i,j));
                    }
                }
            }
            pub_iiwa0_desiredEEInRob.publish(msg_iiwa0_transformd);
            pub_iiwa0_desiredEEInRob_sent.publish(true);
            pub_iiwa1_desiredEEInRob.publish(msg_iiwa1_transformd);
            pub_iiwa1_desiredEEInRob_sent.publish(true);

            rate.sleep();
            exotica_complete = false;
            iiwa0_reached = false;
            iiwa1_reached = false;

            cout << "after publish iiwa_transformd" << endl;

            cout<<"moving iiwa"<<endl;
            while(!exotica_complete || (!iiwa0_reached && !iiwa1_reached)){
                ros::spinOnce();
            }
            cout<<"return true "<<"exotica_complete "<<exotica_complete<<" iiwa0_reached "<<iiwa0_reached<<" iiwa1_reached "<<iiwa1_reached<<endl;
            return true;
        }
        return false;
    }



    if (robotRef == 0)
        cout<<"exotica_complete "<<exotica_complete<<" iiwa0_reached "<<iiwa0_reached<<endl;

    if (robotRef == 1)
        cout<<"exotica_complete "<<exotica_complete<<" iiwa1_reached "<<iiwa1_reached<<endl;

    int64_t nowms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    iiwa0_transformd_file<<nowms<<" ";
    iiwa1_transformd_file<<nowms<<" ";


    //record kuka current transformd
    for (int i = 0; i <3; i++){
        for(int j = 0; j <4; j ++){
            if (robotRef == 0){
                iiwa0_transformd_file<<iiwa0_currentTransformd(i,j)<<" ";
            }
            if (robotRef == 1){
                iiwa1_transformd_file<<iiwa1_currentTransformd(i,j)<<" ";
            }

        }
    }

    //record kuka next transformd
    for (int i = 0; i <3; i++){
        for(int j = 0; j <4; j ++){
            if (robotRef == 0){
                iiwa0_transformd_file<<iiwa_transformd(i,j)<<" ";
            }
            if (robotRef == 1){
                iiwa1_transformd_file<<iiwa_transformd(i,j)<<" ";
            }
        }
    }

    if (robotRef == 0){
        iiwa0_transformd_file<<endl;
    }
    if (robotRef == 1){
        iiwa1_transformd_file<<endl;
    }


    /*if (robotRef == 0){
        while(!iiwa0_reached || !exotica_complete){
            ros::spinOnce;
        }
    }
    if (robotRef == 1){
        while(!iiwa1_reached || !exotica_complete){
            ros::spinOnce;
            //cout<<"exotica_complete "<<exotica_complete<<" iiwa1_reached "<<iiwa1_reached<<endl;
        }
    }*/

    cout << "before publish iiwa_transformd" << endl;

    if (exotica_complete && robotRef == 0){
    //if (iiwa0_reached && exotica_complete && robotRef == 0){
		std_msgs::Float64MultiArray msg;
        for (int i = 0; i <3;i++){
            for (int j = 0; j<4; j++){
                if (j == 3){
                     msg.data.push_back(iiwa_transformd(i,j)/1000.0);
                }else{
                    msg.data.push_back(iiwa_transformd(i,j));
                }
            }
        }
		pub_iiwa0_desiredEEInRob.publish(msg);
        pub_iiwa0_desiredEEInRob_sent.publish(true);
        rate.sleep();
		exotica_complete = false;
		iiwa0_reached = false;		

        cout << "after publish iiwa_transformd" << endl;

        cout<<"moving iiwa"<<endl;
        while(!exotica_complete || !iiwa0_reached){
			ros::spinOnce();
		}
        cout<<"return true "<<"exotica_complete "<<exotica_complete<<" iiwa0_reached "<<iiwa0_reached<<endl;
		return true;		
	}	

    if (exotica_complete && robotRef == 1){
    //if (iiwa1_reached && exotica_complete && robotRef == 1){
		std_msgs::Float64MultiArray msg;
        for (int i = 0; i <3;i++){
            for (int j = 0; j<4; j++){
                if (j == 3){
                     msg.data.push_back(iiwa_transformd(i,j)/1000.0);
                }else{
                    msg.data.push_back(iiwa_transformd(i,j));
                }
            }
        }
		pub_iiwa1_desiredEEInRob.publish(msg);
        pub_iiwa1_desiredEEInRob_sent.publish(true);
        cout<<"iiwa1_desiredEEInRob published"<<endl;
        rate.sleep();
		exotica_complete = false;
		iiwa1_reached = false;		

        cout<<"moving iiwa"<<endl;
        while(!exotica_complete || !iiwa1_reached){
			ros::spinOnce();
            //cout<<"return "<<"exotica_complete "<<exotica_complete<<" iiwa1_reached "<<iiwa1_reached<<endl;

		}
        cout<<"return true "<<"exotica_complete "<<exotica_complete<<" iiwa1_reached "<<iiwa1_reached<<endl;
		return true;	
	}	

    cout << "return false!" << endl;
	return false;
}
