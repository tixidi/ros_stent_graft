#ifndef VISUALTRACKINGTHREAD_H
#define VISUALTRACKINGTHREAD_H

#include <QThread>
#include <QImage>
#include <QTime>
#include <QLabel>
#include <opencv2/opencv.hpp>
#include "needleDrivertracking.h"
#include "definitionsVision.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fstream>
#include <dirent.h>
#include "handEyeAbsoluteOrientation/AbsoluteOrientation.h"
#include "KUKAControl/definitions.h"
#include "KUKAControl/functions.h"
#include <opencv2/video/tracking.hpp>

#include <aruco/aruco.h>
#include <aruco/marker.h>
#include <aruco/markerdetector.h>
#include "tracktools/tracktools.h"
#include "tracktools/mycvdrawingutils.h"
#include "tracktools/mymarkerdetector.h"
#include "VisionSystem/needleTracking.h"
#include "VisionSystem/needlePoseEstimation3Dto2D/needlePoseEstimation.h"

#include "VisionSystem/globals.h"

//string ROBOTFRAME = "../source/VisionSystem/caliInfo/handeyeCaliInput/robotframe.txt";
//string CAMERAFRAME = "../source/VisionSystem/caliInfo/handeyeCaliInput/cameraframe.txt";
const int GRID_1 = 10, GRID_2 = 22, GRID_3 = 14;
const double GRID_SIZE = 0.004224;

//extern string CAMERA_CALI_DIR;

class VisualTrackingThread: public QThread
{
    Q_OBJECT
private:
    stereoCamera camera;
    stereoImages GlobaleImages;
    stereoImages GlobaleImages_show;
    NeedleDriverTracking *trackingNeedleDriver;
    bool findKeyDotPair;
    cv::Mat needleDriverPose;
    bool firstTimeSig;
    ofstream fileMarkerInCameraFrame;
    bool readPosition(char* fname, vector<Point3d> &points);
    bool pointMatch_abs(std::vector<Point3d> &m_R, std::vector<Point3d> &m_C, cv::Mat &R_C);
    bool pointMatch_rigid(std::vector<Point3d> &m_R, std::vector<Point3d> &m_C, cv::Mat &R_C);

    void drawPoints_LeftRight(vector<cv::Point3f> points, int r, int g, int b);
    void drawArrow(cv::Point3f p0, cv::Point3f p1);

    //needle tracker
    NeedlePoseEstimation *needleTracking;
    IplImage *needleEngeryL, *needleEngeryR;
    cv::Mat needleEnergyL_mat, needleEnergyR_mat;
    vector<double> needleEnergyPixel_L, needleEnergyPixel_R;

    QTime *disPlayTime;
    //record video
    cv::VideoWriter videoL, videoR;
        bool sigREcod;

    // Draw needle driver frame
    stereoImages showNeedleDriverFrameInLeft(stereoImages &Images);

    // Tools tracking by myaruco
    //myaruco::TrackTools *toolsTracker;
    int numTools;
    int fCOUNTER;

    //Mat ToolLHNeedle, ToolLHNeedleTip, ToolRHTipL, ToolRHTipR, ToolRHNeedle, ToolRHNeedleTip;
    Mat ToolLHNeedleC;

    int countFrames;

    // Draw force direction
    void drawThreadForce(double x, double y, double z, int nx, int ny, int nz);

    // Inspection
    string ename_needleEnergyL, ename_needleEnergyR;
    void findThreadInImages(Point3d pnt3_root, Point3d &pnt3_tip);
    void Point3ToPoint2(vector<Point3d> &pnt3, vector<Point2d> &pnt2_l, vector<Point2d> &pnt2_r);
    void saveAllNeedleResults(vector<vector<cv::Point3d> > needlePoints_all, needlePoseEstimation *pose3Dto2D);


public:
    VisualTrackingThread();
    void run();
    bool saveStereoImages(int imageRef, string l, string fname_r);
    bool saveStereoImages2(int a);
    bool saveChessboardImages(int imageRef, string fname_l, string fname_r);
    //bool StereoCalib();
    bool calculateAndSaveMarkerPoseInCameraFrame();
    bool handeyeCali_abs(char* fname_mR, char* fname_mC, cv::Mat &R_C);
    bool detectMarkerInCamFrame( Matrix4d & MarkerInCamFrame);
    bool detectMarkerInCamFrame_rect( Matrix4d & MarkerInCamFrame, bool needRectify);

    void showNeedleDriverFrameInCam( cv::Mat &needleDriverPoseInCamCV, bool newhandye );
    void showNeedleInCam( cv::Mat &needlePoseInCamCV, std::vector<cv::Point3d> &needle3Dmodel );

    bool getRelaPoseNeedleAndNeedleDriver(cv::Mat &robotTipPose ,Matrix4d &newNeedlePose);
    bool getNeedleDriverMarkerPoseInCamFrame(cv::Mat &Pose );
    bool getMarkerInCameraFromHandEye(cv::Mat &EEPostureCV, cv::Point3d &M2C_handeye);
    vector<vector<vector<cv::Point3d> > > grid_robot;
    vector<vector<vector<cv::Point3d> > > grid_camera;
    std::string  initializeRecodeVideo(string fileDir , char *buffer);
    void recordVideo();
    void drawLine3Dto2D(cv::Mat thread);


    cv::Mat needleDriverPoseInCamerma;

    // Camera parameters
    cv::Mat M1, D1, mx1, my1, P1;
    cv::Mat M2, D2, mx2, my2, P2;

    // Tool tracking ----------------
    myaruco::TrackTools *toolsTracker;
    bool DrawTool, DrawNeedle;
    //cv::Mat ToolPoseInCamera, ToolPoseInCamera_handeye;
    //aruco::Marker ToolPoseInCamera, ToolPoseInCamera_handeye;
    void initTools(vector<float> mSizes, vector<vector<int> > tools_ID);
    bool detectToolsInCamFrame(vector<bool> &toolDetected);
    bool detectToolsInCamFrame_global(Mat3b frameL, Mat3b frameR);

    // Filter
    int filterWindow;
    vector<vector<cv::Mat> > toolsTvec_hist, toolsRvec_hist;

    // Hand eye
    cv::Mat markerInHandEye, markerInHandEye_updated;
    bool DrawHandEye;
    void showHandEye(Mat3b frame, cv::Mat cHr);

    // Draw trajectory ---------------
    bool DrawTrajectory, DrawTrajectoryToolL, DrawTrajectoryToolR, DrawMandrel, DrawTrajectoryNeedle, DrawThread;
    vector<Point3f> toolLTraj_tvec, toolRTraj_tvec, mandrel_tvec, needleTraj_tvec;
    vector<Point3f> toolLTraj_rvec, toolRTraj_rvec;
    cv::Mat threadPoints;
    void drawTrajectory(vector<Point3f> trajectory, int r, int g, int b);
    void drawNeedleTrajectory(vector<Point3f> toolTraj, vector<Point3f> toolTraj_Rvec, int r, int g, int b);
    void drawTipTrajectory(vector<Point3f> toolTraj, vector<Point3f> toolTraj_Rvec, int r, int g, int b);
    bool detectNeedle, needleDetected;
    bool saveDetecedNeedle;
//    vector<cv::Point3d> NeedlePoints_cal;

    // Save image -------------------
    bool SaveImage;
    int imageInd;

    int Traj_index;
    bool PutText;

    // Timer ------------------------
    bool bool_recordTime;
    time_t timer;
    ofstream toolsInCamTimer;
    int imageCnt;

    // Needle ---------------------------
    ofstream fneedleNewInToolL, fneedleNewInToolR;
    Mat tHn_;
    vector<cv::Point3d> NeedlePoints, NeedlePoints1, NeedlePoints2;
    Mat ToolLHNeedle, ToolLHNeedleTip, ToolRHTipL, ToolRHTipR, ToolRHNeedle, ToolRHNeedleTip;
    int toolIndx_detectneedle;

    // Force ----------------------------
    bool DrawForce;
    int forceX, forceY, forceZ;


    // Inpect images: detection results
    bool Inspect;
    char* fname_thread_ori, *fname_thread_new;

    bool drawSlots;
    vector<Matrix4d> MAN_H_SLOTS_NEW_draw;
    void showSlots();
    int Curr_Slot;

signals:
    void imagesReady(stereoImages Images);
    void showImagesInDialog(stereoImages Images);
    void showThreadInDialog(stereoImages Images);


public slots:
    void checkImages();
    void displayThread();
    void showThread(int indx);
};
#endif
