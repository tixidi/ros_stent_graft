#ifndef KUKAMOTIONPLANNING_H
#define KUKAMOTIONPLANNING_H

#include <iostream>     // std::cout
#include <fstream>      // std::ifstream

#include <QThread>
#include <QTimer>
#include <QtCore>
#include <QTime>
#include <QApplication>
#include <QGraphicsScene>
#include <QLabel>
#include <QDialog>
#include <QMessageBox>
#include <QDebug>

#include "KUKAControl/definitions.h"
#include "KUKAControl/DualArmRobot.h"
#include "KUKAControl/functions.h"
#include "config.h"
#include "iiwaControl/iiwaControl.h"

#ifdef VisionSystemON
#include "VisionSystem/mainWindowVision.h"
#include "VisionSystem/visualTrackingThread.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/v4l2-mediabus.h>
#endif

#include <Eigen/Dense>
#include "VisionSystem/globals.h"


#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include "sensor_msgs/JointState.h"
#include <iiwa_test/iiwaState.h>
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif


class KukaMotionPlanning: public QThread
{
   Q_OBJECT
public:
   KukaMotionPlanning();
   void setOpMode(int opMode);
   void onAndOffButton(bool);
  // Matrix4d markerTransMap2TipTrans(int markRef, Matrix4d &trans);
   //void saveEndeffecterPoseAndHanfEyeTrans(int robotRef);]
   Matrix4d readToolPoseInIIWAbase(int robotRef, char *toolFile);
//    Matrix4d readMarkerPoseInBaseFrame(int markerRef, int robotRef, ifstream &f_stream3);
   /*
   Matrix4d readToolPoseInBaseFrame(int robotRef, char *toolFile);*/
   void computeToolInManTraj(vector<robotPosture> &traj_MandrelInCam,
                                                 vector<robotPosture> &traj_ToolLInCam,
                                                 vector<robotPosture> &traj_ToolLInMan);
   void tranTool2Robot(Matrix4d manInRob_trans, vector<robotPosture> &learntTraj, vector<robotPosture> &toolInRob);
   void tranToolInRob2NeeldeInRob(Matrix4d toolInEE_trans, vector<robotPosture> &toolInRob, vector<robotPosture> &EEInRob);
   void readMandrelToolsPoses( char * fileDir, std::vector<robotPosture> &mandrel , std::vector<robotPosture> &toolL, vector<robotPosture> &toolR);
   void readMandrelToolsDrivers( char * fileDir, vector<robotPosture> &mandrel,
                                 vector<robotPosture> &toolL, vector<robotPosture> &toolR,
                                 vector<float> &driverL, vector<float> &driverR);
   void readMandrelToolsDriversInCamera(char * fileDir, vector<cv::Point3f> &mandrel,
                                         vector<cv::Point3f> &toolL_tvec, vector<cv::Point3f> &toolR_tvec,
                                         vector<Point3f> &toolL_rvec, vector<Point3f> &toolR_rvec);

   bool visionGuidedMoveToPosture_newvision(robotPosture currentToolInMan,
                                          robotPosture targetToolInMan, int robotIndex,
                                          Matrix4d robInCameraFrame,
                                          Matrix4d toolInEE, char *toolFile);

   bool visionGuidedMoveToPosture_kalmanvision_iiwa(robotPosture currentToolInMan,
                                                      robotPosture targetToolInMan, int robotIndex,
                                                      Matrix4d robInCameraFrame,
                                                      Matrix4d toolInEE, char *toolFile,
                                                      char *millisecbuffer, int ms);

   bool moveToEEPose(robotPosture EEInRob_des, int robotIndex);
   bool pullThread_moveToEEPose( robotPosture EEInRob_des, int robotIndex, int force_thres);

   bool visionGuidedMoveToPosture_noVision_iiwa(robotPosture currentToolInMan,
                                                  robotPosture targetToolInMan, int robotIndex,
                                                  Matrix4d robInCameraFrame,
                                                  Matrix4d toolInEE, char *toolFile, char *millisecbuffer, int ms);

   robotPosture visionGuidedIIWAMoveToInitialPosture(robotPosture targetToolInMan, int robotIndex,
                                          Matrix4d robInCameraFrame,
                                          Matrix4d toolInEE, char *toolFile);

   bool readToolsTrajinMandrelandDrivers(int robotIndex,
                                           vector<robotPosture> &EEInRobot,
                                           vector<robotPosture> &traj_ToolInMan,
                                           vector<float> &traj_Driver,
                                           char* r2cFile, char* toolFile, char* trajectFile);
   void transferToolsTrajByNeedlePose(vector<robotPosture> &EEInRobot,
                                      vector<robotPosture> &traj_ToolInMan,
                                      vector<robotPosture> &traj_ToolInCamera,
                                      vector<cv::Point3f> &tool_tvec, vector<Point3f> &tool_rvec,
                                      int size_seg, Matrix4d tHn, Matrix4d tHn_,
                                      char* r2cFile, char* toolFile, char* trajectFile
                                      );
   void checkQuaternion(Eigen::Quaterniond &quad_curr, Eigen::Quaterniond &quad_next);

private:
   int SampleTime ;
   void run();
   DualArmRobot *dualArmPlanner;
   iiwaControl *iiwa_0;
   iiwaControl *iiwa_1;
   vector<iiwaControl*> iiwas;
//   forceSensor *forceReading;


   Vector6d targetPosOri;
   void pathPlanning();
//   void bimanualSewing();
   void calibRobot();
   int controlopMode;
   bool onAndOff;
   kukaInformation kukaStates;
   QTime *timeCnt;
   VisualTrackingThread *myTracker;

   //  for handeysCali
   ofstream imageList, fileMarkPoseRob0;
   ofstream f_ee_mat, f_marker2robot, f_marker2camera, f_marker2handeye, f_ee_pose;
   ofstream f_robotJoints0;
   int handEyeCaliStatus;

   //Configureation files; Trajectory files-----------------
   char* fname_thread_ori, fname_thread_new;
   std::string fname_traj_toolsInCam, fname_traj_EEInBase, fname_hist_HandEye0_,
               fname_traj_tool_Timer, fname_traj_EEInBase_Timer, fname_joints_0,
               fname_kuka0Hee, fname_kuka_joints_traj;
   std::string dir_demo;

   ofstream toolsInCam, fEEInRobot0, fEEInRobots, fHandEye0, fHandEye1, fEEInRobots_timer, fJoints01, fkuka0Hee, fKukaJointsTraj;
   // ROS --------------------------------
    int rate_hz = 300;
    bool exotica_complete = false;
	bool iiwa0_reached = false;
	bool iiwa1_reached = false;
	bool iiwa0_connected = false;
	bool iiwa1_connected = false;
    bool iiwa0_msrTransform_received = true;
    bool iiwa1_msrTransform_received = true;
    int iiwa0_state_count = -1;
    int iiwa1_state_count = -1;
	Matrix4d iiwa0_currentMartix4d, iiwa1_currentMartix4d;
	Erl::Transformd iiwa0_currentTransformd, iiwa1_currentTransformd;
	//Eigen::Matrix<double, 7, 1> iiwa0_currJoints, iiwa1_currJoints;
	//bool iiwa0_desiredEEInRob_published = false;
	//bool iiwa1_desiredEEInRob_published = false;



    void posCallback_iiwa0_state(const iiwa_test::iiwaState::ConstPtr& msg);
    void posCallback_iiwa1_state(const iiwa_test::iiwaState::ConstPtr& msg);
	void posCallback_iiwa0_reached(const std_msgs::Bool::ConstPtr& msg);
	void posCallback_iiwa1_reached(const std_msgs::Bool::ConstPtr& msg);
	void posCallback_iiwa0_connected(const std_msgs::Bool::ConstPtr& msg);
	void posCallback_iiwa1_connected(const std_msgs::Bool::ConstPtr& msg);
	void posCallback_exotica_complete(const std_msgs::Bool::ConstPtr& msg);
	//void posCallback_iiwa0_currJoints(const sensor_msgs::JointState::ConstPtr& msg);
	//void posCallback_iiwa1_currJoints(const sensor_msgs::JointState::ConstPtr& msg);
	
    bool moveIiwa(int robotRef, Erl::Transformd iiwa_transformd);
   // IIWA -------------------------------
   int Num_Slot;
   double Dist_Slot;
   int Curr_Slot;

   //FOR demo ---------------------------------------------------------------------------
   ofstream recordTrajectory;
   bool INIT_STATUS, INIT_SEW;
   void RobotPause(int robotIndex);
   bool moveToEEPose_QuatSave(int robotIndex, robotPosture EEInRobot);
   void readEETraj(char * fileDir, vector<robotPosture> &EE);
   void InitializeTrajectory(int status, char* trajFile, cv::Mat &adapt_mat,
                             bool moveToBase, bool moveToCam,
                             vector<robotPosture> &traj_manIncam,
                             vector<robotPosture> &traj_toolInman,
                             vector<float> &traj_driverl, vector<float> &traj_driverr, bool startRecording);
   void PlayTrajectory_CameraFrame(int robotIndex, int trajIndex_pre, int trajIndex,
                                    vector<robotPosture> traj_toolInman,
                                    vector<float> traj_driverl, vector<float> traj_driverr,
                                    char* millisecbuffer, int ms);

//   NeedleDriverInterface *myNeedleDriver;
   char millisecbuffer [80];
   char buffer [80];
   long int ms;
   int NUM_CYCLE;



   cv::Mat manInCam_ini;
//    Matrix4d handeye_ini;
   Matrix4d initToolInRobFrame;
   bool initTool_bool;

   Matrix4d iniToolLHNeedle_ei, iniToolRHNeedle_ei;

   // For thread ---------------
   Mat threadTransform(Mat thread_old, Mat thread_new, Point3d tool_old, Point3d tool_new);
   void computeThreadFrame(Mat thread_, Point3d tool_, Mat &frame_);
   void adaptToolRbyThread(vector<robotPosture> &traj_ToolRInCam);

   // For updating hand-eye -------------------------------
   int handeye_window, handeye_count0, handeye_count1;
   Mat cHr0_updated, cHr1_updated;
   vector<robotPosture> robot0InCamera_hist, robot1InCamera_hist;
   void UpdateHandEye(robotPosture robotInCam, int robotIndex);

   // Timer ----------------
   time_t timer;

   // Kalman filter --------------------
   vector<robotPosture> hist_EEInRobotBase;
   int cnt_robothist;
   int CAMERALATENCY, ROBOTSLEEP, TRAJSTEP;
   Mat R_kalman, Q_kalman, P_kalman, K_kalman;

   void recordRobotPose(vector<robotPosture> &EEInRobot, char *fname);
   void record3DPose(vector<cv::Point3f> &Tvec, char *fname);

   // Configuration -----------------
   char *fname_cHr0, *fname_cHr1, *fname_eeHl, *fname_eeHr, *fname_cTi, *fname_eTm, *fname_mHm_, *fname_eeHs;
   char *fname_ee_mat, *fname_robotHmarker_mat, *fname_cameraHmarker_mat, *fname_cameraHmarker_handeye_mat, *fname_ee_robpos, *fname_robotJoints0;
//   string fname_cHr0, fname_cHr1, fname_eeHl, fname_eeHr, fname_cTi, fname_eTm, fname_mHm_, fname_eeHs;
//   string fname_ee_mat, fname_robotHmarker_mat, fname_cameraHmarker_mat, fname_cameraHmarker_handeye_mat, fname_ee_robpos, fname_robotJoints0;
   //ifstream fstream_cHr0, fstream_cHr1, fstream_eeHl, fstream_eeHr;
   cv::Mat CAMERA_H_ROBOT0, CAMERA_H_ROBOT1, EE_H_TOOLL, EE_H_TOOLR;

   // Dialogs --------------------
//   mainWindowInspection *Inspection;
   bool inspection_answer;
   bool inspection_clicked;
   string ename_needleEnergyL, ename_needleEnergyR;

//   mainWindowInspection *DispThread;
//    bool displaythread_clicked, displaythread_answer;



public slots:
   void plannerStop();
   void plannerStart();
   void plannerClose();
   void recordforLiang();

   void enableGravy();
   void disableGravy();

   void samplingKukaStares();
   void inspection_yes_clicked();
   void inspection_no_clicked();

signals:
   void kukaInformAvaiable(kukaInformation);
   void checkImages();
   void checkThread();
   void detectThread(int no_slot);
};
#endif
