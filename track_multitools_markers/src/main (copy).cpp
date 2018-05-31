#include <ros/ros.h>
#include <track_multitools_markers/ToolsPose.h> 
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
//#include <opencv2/highgui/highgui.hpp>
#include "mycvdrawingutils.h"
#include "mymarkerdetector.h"
//#include <opencv2/video/tracking.hpp>
#include "patterntracker/tracker_general.h"
#include "patterntracker/tracker.h"
#include <unistd.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/v4l2-mediabus.h>

using namespace cv;
using namespace aruco;
using namespace myaruco;
using namespace dvrk;

int CameraL_ind, CameraR_ind;
string TheInputVideo;
string theIntrinsicFileL, theIntrinsicFileR;
int streamMode;
string videoFnameL, videoFnameR;
float TheMarkerSize=-1;
int ThePyrDownLevel;
myaruco::MarkerDetector MDetector;
myaruco::MarkerDetector MDetectorL, MDetectorR;
VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
vector<Marker> TheMarkersL, TheMarkersR;
vector<Marker> StereoMarkersL, StereoMarkersR, StereoMarkersL_pre;
vector<Marker> ToolMarkersL, ToolMarkersR;
Marker  ToolL_pre, ToolL_filter;
Marker  ToolR_pre, ToolR_filter;
Marker  ToolM_pre, ToolM_filter;
Marker ToolL, ToolR, Mandrel;
Mat Q, mx1, my1, mx2, my2, P1, P2, T, R, D1, D2;
vector<Point3f> markercornerLocal;

Mat TheInputImage,TheInputImageCopy;
CameraParameters TheCameraParameters;
CameraParameters TheCameraParametersL, TheCameraParametersR;
CameraParameters StereoCameraParametersL, StereoCameraParametersR;

KalmanFilter KFL(6,3,0), KFL_rot(6,3,0);
KalmanFilter KFR(6,3,0), KFR_rot(6,3,0);
KalmanFilter KFM(6,3,0), KFM_rot(6,3,0);

Mat ToolLHNeedle, ToolLHNeedleTip, ToolRHTipL, ToolRHTipR, ToolRHNeedle, ToolRHNeedleTip;

pair<double,double> AvrgTime(0,0) ;//determines the average time required for detection
double ThresParam1,ThresParam2;
int iThresParam1,iThresParam2;
int waitTime=10;


void tuneFocus();
const int FOCUSABSOLUTE=40;
#define CAMERA0_DEV "/dev/video0"
#define CAMERA1_DEV "/dev/video1"
/************************************
 *
 *
 *
 *
 ************************************/
bool readArguments ( int argc,char **argv );
void findStereoMarkers();
void findStereoMarkers_tracker(vector<Marker> markersLL, vector<bool> markersLL_bool,
                               vector<Marker> markersRL, vector<bool> markersRL_bool);
void estimateStereoMarkerPoses();
void kalmanFilterPose();
bool estimateToolPose(Marker &Tool, const int toolID[], const int N, const float tsize);
bool estimateToolPose_singlemarker(aruco::Marker &tool, vector<int> toolID, int N, float markersize);
bool estimateToolPose_multimarkers(Marker &Tool, const int toolID[], const int N, const float tsize);
bool estimateMandrelPose(Marker &man, const int manID[], const int N, const float mRadius);
vector<bool> trackMarkers(cv::Mat &track_image, vector<Marker> &detectedMarkers,
                  vector<Marker> &toolMarkers, vector<TrackerGeneral> &trackers, CameraParameters camParams, float markerSizeMeters);
void drawNeedleFnt(cv::Mat3b frame, bool bool_frameL, cv::Mat ToolHNeedle, cv::Mat ToolHNeedleTip, int toolIndex);


const int tool_N = 5;
//const int ToolID_L[tool_N] = {390, 267, 184, 399, 437};
//const int ToolID_L[1] = {717};
int tmp[] = {717};
vector<int> ToolID_L(tmp, tmp+1);
const int ToolID_R[tool_N] = {289, 239, 189, 876, 654};
//const int ToolID_R[tool_N] = {918, 2, 384, 834, 993};

//const float Tsize = 0.0109;
float Tsize = 0.0109;
const float Ttorlerance = 0.001;

//const int mandrel_N = 4;
//const int MandrelID[mandrel_N] = {321, 717, 221, 62};
//const float mRadius = 0.044;S
const int mandrel_N = 8;
const int MandrelID[mandrel_N] = {413, 321, 97, 819, 123, 221, 928, 62,};
const float mRadius = 0.066;
const float mSize = mRadius * tan(M_PI/mandrel_N);


void initializeKF(KalmanFilter &kf, KalmanFilter &kf_rot);
void kalmanFilterPose(Marker &pose_detected, Marker &pose_previous, Marker &pose_filtered,
                      KalmanFilter &KF, KalmanFilter &KF_rot, bool &bool_fristDetection);
void cvTackBarEvents(int pos,void*);
bool readCameraParameters(string TheIntrinsicFile,CameraParameters &CP,Size size);
template <typename CV_POINT3_SRC, typename CV_POINT3_DST>
static void find_rigidTransformation(const std::vector<CV_POINT3_SRC> &_src,
                                     const std::vector<CV_POINT3_DST> &_dst,
                                     cv::Mat &H_src2dst);

bool DrawNeedle = false;
bool DrawTool = true;
bool BOOL_TRACK = true;

const int fabric_N = 4;
const int fabricID[fabric_N] = {97, 457, 717, 62};
bool findMarkerPose_byID(Marker &fabric, int id);

/************************************
 *
 *
 *
 *
 ************************************/
int main(int argc,char **argv)
{

		//Initiate new ROS node and setup a node
		ros::init(argc, argv, "track_multitools_markers");
    ros::NodeHandle nh; 
    ros::Publisher pub=nh.advertise<track_multitools_markers::ToolsPose>("tools_pose", 100);
    srand(time(0));
    ros::Rate rate(10);
		track_multitools_markers::ToolsPose msg;

    ofstream myfile;
    //myfile.open("toolsmandrel.txt");

    try
    {
        if (readArguments (argc,argv)==false)
        {
            return 0;
        }
        //parse arguments
        ;

        // Write to trajactory file / videos-----------------------------------------
        bool bool_write = false;
        bool bool_write_raw = false;
        cv::VideoWriter videoL, videoR;

        //Kalman filter ---------------------------------------------

        // initialization of kf --------------------------
        bool bool_initKFL = false;
        bool bool_initKFR = false;
        bool bool_initKFM = false;

        initializeKF(KFL, KFL_rot);
        initializeKF(KFR, KFR_rot);
        initializeKF(KFM, KFM_rot);
        // ==========================


        // initialization of tools ----------------------
        ToolL.Tvec.ptr<float>(0)[0] = -1000;
        ToolL.Tvec.ptr<float>(0)[1] = -1000;
        ToolL.Tvec.ptr<float>(0)[2] = -1000;
        ToolL.Rvec.ptr<float>(0)[0] = -1000;
        ToolL.Rvec.ptr<float>(0)[1] = -1000;
        ToolL.Rvec.ptr<float>(0)[2] = -1000;

        // initialize marker L array
        vector<Marker>  MarkersLL; // left view Tool L
        vector<Marker>  MarkersRL; // right view Tool L
        Marker tempMarker;
        tempMarker.Tvec.ptr<float>(0)[0] = -1000;
        tempMarker.Tvec.ptr<float>(0)[1] = -1000;
        tempMarker.Tvec.ptr<float>(0)[2] = -1000;
        tempMarker.Rvec.ptr<float>(0)[0] = -1000;
        tempMarker.Rvec.ptr<float>(0)[1] = -1000;
        tempMarker.Rvec.ptr<float>(0)[2] = -1000;

        for (int i=0; i<tool_N; i++)
        {
            tempMarker.id = ToolID_L[i];
            MarkersLL.push_back(tempMarker);
            MarkersRL.push_back(tempMarker);
        }
        // ===============================================


        // initialization of trackers ------------------------------------
        vector<cv::Point2f> markercornerLocal;
        markercornerLocal.resize(4);
        double halfSize=TheMarkerSize/2.;
        markercornerLocal[0].x = -halfSize;
        markercornerLocal[0].y = -halfSize;
        markercornerLocal[1].x = -halfSize;
        markercornerLocal[1].y = halfSize;
        markercornerLocal[2].x = halfSize;
        markercornerLocal[2].y = halfSize;
        markercornerLocal[3].x = halfSize;
        markercornerLocal[3].y = -halfSize;

        vector<dvrk::TrackerGeneral> trackersLL, trackersRL;
        for (int i=0; i<tool_N; i++)
        {
            dvrk::TrackerGeneral traker_tmp(markercornerLocal);
            trackersLL.push_back(traker_tmp);
            trackersRL.push_back(traker_tmp);
        }
        // ===============================================================



        // Read needle to tool pose -----------------------------
       ToolLHNeedle = Mat::eye(4,4,CV_64F);
       ifstream f_streamlHn("../dummy.txt");
         for (int i=0; i<4; i++)
             for (int j=0; j<4; j++)
                 {
                     double variable;
                     f_streamlHn >> variable;
                     ToolLHNeedle.at<double>(i,j)=variable;
                 }
         cout << "ToolLHNeedle " << ToolLHNeedle << endl;

         ToolLHNeedleTip = Mat::eye(4,4,CV_64F);
         ifstream f_streamlHt("../dummy.txt");
           for (int i=0; i<4; i++)
               for (int j=0; j<4; j++)
                   {
                       double variable;
                       f_streamlHt >> variable;
                       ToolLHNeedleTip.at<double>(i,j)=variable;
                   }
           cout << "ToolLHNeedleTip " << ToolLHNeedleTip << endl;

           ToolRHTipL = Mat::eye(4,4,CV_64F);
           ifstream f_streamRHl("../dummy.txt");
             for (int i=0; i<4; i++)
                 for (int j=0; j<4; j++)
                     {
                         double variable;
                         f_streamRHl >> variable;
                         ToolRHTipL.at<double>(i,j)=variable;
                     }
             //cout << "ToolRHTipL " << ToolRHTipL << endl;

             ToolRHTipR = Mat::eye(4,4,CV_64F);
             ifstream f_streamRHr("../dummy.txt");
               for (int i=0; i<4; i++)
                   for (int j=0; j<4; j++)
                       {
                           double variable;
                           f_streamRHr >> variable;
                           ToolRHTipR.at<double>(i,j)=variable;
                       }
               //cout << "ToolRHTipR " << ToolRHTipR << endl;

           ToolRHNeedle = Mat::eye(4,4,CV_64F);
           ifstream f_streamRHn("../dummy.txt");
             for (int i=0; i<4; i++)
                 for (int j=0; j<4; j++)
                     {
                         double variable;
                         f_streamRHn >> variable;
                         ToolRHNeedle.at<double>(i,j)=variable;
                     }
             cout << "ToolRHNeedle " << ToolRHNeedle << endl;

             ToolRHNeedleTip = Mat::eye(4,4,CV_64F);
             ifstream f_streamRHt("../dummy.txt");
               for (int i=0; i<4; i++)
                   for (int j=0; j<4; j++)
                       {
                           double variable;
                           f_streamRHt >> variable;
                           ToolRHNeedleTip.at<double>(i,j)=variable;
                       }
               cout << "ToolRHNeedleTip " << ToolRHNeedleTip << endl;
          // =========================================================================================



        // Open cameras
        VideoCapture cameraL, cameraR;
        cv::Mat3b frameL, frameR, frameL_copy, frameR_copy;
        switch (streamMode)
        {
            case 0:
                cout << "Running camera!" << endl;
                cameraL.open(CameraL_ind); cameraR.open(CameraR_ind);
                if (!cameraL.isOpened()) return -1;
                if (!cameraR.isOpened()) return -1;
                tuneFocus();
                cameraL.set(CV_CAP_PROP_FPS,30);
                cameraR.set(CV_CAP_PROP_FPS,30);

                cameraL >> frameL;
                cameraR >> frameR;
                break;
            case 1:
                cout << "Reading video (non-rectified)!" << endl;
                cameraL.open(videoFnameL); cameraR.open(videoFnameR);
                if (!cameraL.isOpened()) return -1;
                if (!cameraR.isOpened()) return -1;
                cameraL >> frameL;
                cameraR >> frameR;
                break;
            case 2:
                frameL = imread("left1.jpg", CV_LOAD_IMAGE_COLOR);
                frameR = imread("right1.jpg", CV_LOAD_IMAGE_COLOR);
                break;
            default:
                cout << "please enter stream mode. 0:camera 1:video 2:photo" << endl;
        }

        //read first image to get the dimensions

        Mat P1_ = P1(cv::Rect(0,0,3,3));
        P1_.copyTo(StereoCameraParametersL.CameraMatrix);
        D1.copyTo(StereoCameraParametersL.Distorsion);

        //Configure other parameters
        if (ThePyrDownLevel>0)
            MDetector.pyrDown(ThePyrDownLevel);


        //Create gui

        //cv::namedWindow("thres",1);
        cv::namedWindow("left",1);
        MDetectorL.getThresholdParams( ThresParam1,ThresParam2);
        MDetectorL.setCornerRefinementMethod(myaruco::MarkerDetector::LINES);
        MDetectorR.getThresholdParams( ThresParam1,ThresParam2);
        MDetectorR.setCornerRefinementMethod(myaruco::MarkerDetector::LINES);
        iThresParam1=ThresParam1;
        iThresParam2=ThresParam2;
        cv::createTrackbar("ThresParam1", "in",&iThresParam1, 13, cvTackBarEvents);
        cv::createTrackbar("ThresParam2", "in",&iThresParam2, 13, cvTackBarEvents);

        char key=0;
        int index = 0;

        //capture until press ESC or until the end of the video
        while ( key!=27 || ros::ok())
        {


            fstream f_stream("test.txt");
            f_stream >> Tsize;

            //copy image
            if (streamMode==0 || streamMode==1)
            {
                cameraL >> frameL;
                cameraR >> frameR;
                //Rectify image
                remap(frameL, frameL_copy, mx1, my1, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );
                remap(frameR, frameR_copy, mx2, my2, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );                
                index++;
            }

            //Detection of markers in the image passed
            MDetectorL.detect(frameL_copy,TheMarkersL,TheCameraParametersL,TheMarkerSize);
            MDetectorR.detect(frameR_copy,TheMarkersR,TheCameraParametersR,TheMarkerSize);

            //print marker info and draw the markers in image
            //frameL.copyTo(frameL_copy);
            for (unsigned int i=0;i<TheMarkersL.size();i++) {
                //cout<<TheMarkers[i]<<endl;
                TheMarkersL[i].draw(frameL_copy,Scalar(0,0,255),1);
            }
            //frameR.copyTo(frameR_copy);
            for (unsigned int i=0;i<TheMarkersR.size();i++) {
                //cout<<TheMarkers[i]<<endl;
                TheMarkersR[i].draw(frameR_copy,Scalar(0,0,255),1);
            }

            if (streamMode==1)
            {
                char text[255];
                sprintf(text, "%d", index);
                int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
                double fontScale = 1;
                int thickness = 3;
                cv::Point textOrg(500, 440);
                cv::putText(frameL_copy, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness,8);
                cv::putText(frameR_copy, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness,8);
            }

            //Track Markers -------------------------------------------------
            /*
             *
             *
             *
             ---------------------------*/
            //trackMarkers(TheMarkersL, MarkersLL);
            //trackMarkers(TheMarkersR, MarkersRL);
           // trackMarkers(frameL, TheMarkersL, MarkersLL, trackersLL);
            vector<bool> trakerMarksLL_bool = trackMarkers(frameL, TheMarkersL,
                                                           MarkersLL, trackersLL,
                                                           TheCameraParametersL,TheMarkerSize);
            for (unsigned int i=0;i<MarkersLL.size();i++)
            {
                if (trakerMarksLL_bool[i])
                {
                    //cout << MarkersRL[i][0] << endl;
//                   MarkersLL[i].draw(frameL_copy,Scalar(255,0,0),1);
                }
            }

            vector<bool> trakerMarksRL_bool = trackMarkers(frameR, TheMarkersR,
                                                           MarkersRL, trackersRL,
                                                           TheCameraParametersR,TheMarkerSize);
            for (unsigned int i=0;i<MarkersRL.size();i++)
            {
                if (trakerMarksRL_bool[i])
                {
                    //cout << MarkersRL[i][0] << endl;
//                    MarkersRL[i].draw(frameR_copy,Scalar(255,0,0),1);
                }
            }


            //Stereo cameras -------------------------------------------------
            /*
             *
             *
             *
             ---------------------------*/

            //Find markers showing in both cameras
            StereoMarkersL.clear();
            StereoMarkersR.clear();
            ToolMarkersL.clear();
            ToolMarkersR.clear();

            //findStereoMarkers();
            findStereoMarkers_tracker(MarkersLL,trakerMarksLL_bool,MarkersRL,trakerMarksRL_bool);
            estimateStereoMarkerPoses();

            //draw a 3d cube in each marker if there is 3d info
            for (unsigned int i=0;i<StereoMarkersL.size();i++)
            {                
                StereoMarkersL[i].draw(frameL_copy,Scalar(0,255,0),1);
            }


            //Detect tool ---------------------------------------------
            /*
             *
             *
             *
             *
             * ======================================================= */

            bool ToolDetectedL = false, ToolDetectedR = false, MandrelDetected = false;
            if (StereoMarkersL.size()>0)
            {
                ToolDetectedL = estimateToolPose_singlemarker(ToolL, ToolID_L, tool_N, Tsize);
                ToolDetectedR = estimateToolPose_multimarkers(ToolR, ToolID_R, tool_N, Tsize);
                MandrelDetected = estimateMandrelPose(Mandrel, MandrelID, mandrel_N, mRadius);
            }

            if (ToolDetectedL)
            {
                if (bool_initKFL)
                {
                    ToolL.Tvec.copyTo(ToolL_pre.Tvec);
                    ToolL.Rvec.copyTo(ToolL_pre.Rvec);
                }
//                kalmanFilterPose(ToolL, ToolL_pre, ToolL_filter, KFL, KFL_rot, bool_initKFL);
                ToolL_filter.Tvec.copyTo(ToolL_pre.Tvec);
                ToolL_filter.Rvec.copyTo(ToolL_pre.Rvec);
                ToolL_pre.ssize = ToolL.ssize;

                if (DrawTool)
                CvDrawingUtils::draw3dAxis(frameL_copy, ToolL, StereoCameraParametersL);
            }
            else if(!bool_initKFL)
            {
                if (DrawTool)
                    CvDrawingUtils::draw3dAxis(frameL_copy, ToolL, StereoCameraParametersL);
            }

            //DrawNeedle = true;
            if(DrawNeedle)
            {
                drawNeedleFnt(frameL_copy, true, ToolLHNeedle, ToolLHNeedleTip,1);
                drawNeedleFnt(frameR_copy, false, ToolLHNeedle, ToolLHNeedleTip,1);
            }


            if (MandrelDetected)
            {
                if (bool_initKFM)
                {
                    Mandrel.Tvec.copyTo(ToolM_pre.Tvec);
                    Mandrel.Rvec.copyTo(ToolM_pre.Rvec);
                }
//                kalmanFilterPose(Mandrel, ToolM_pre, ToolM_filter, KFM, KFM_rot, bool_initKFM);
                ToolM_filter.Tvec.copyTo(ToolM_pre.Tvec);
                ToolM_filter.Rvec.copyTo(ToolM_pre.Rvec);
                ToolM_pre.ssize = Mandrel.ssize;

                if (DrawTool)
                    CvDrawingUtils::draw3dAxis(frameL_copy, Mandrel, StereoCameraParametersL);
//                cout << "mandrel: " << Mandrel.Tvec << " " << Mandrel.Rvec << endl;
            }
            else if(!bool_initKFM)
            {
                if (DrawTool)
                    CvDrawingUtils::draw3dAxis(frameL_copy, Mandrel, StereoCameraParametersL);
            }

            if (ToolDetectedR)
            {
                if (bool_initKFR)
                {
                    ToolR.Tvec.copyTo(ToolR_pre.Tvec);
                    ToolR.Rvec.copyTo(ToolR_pre.Rvec);
                }
//                kalmanFilterPose(ToolR, ToolR_pre, ToolR_filter, KFR, KFR_rot, bool_initKFR);
                ToolR_filter.Tvec.copyTo(ToolR_pre.Tvec);
                ToolR_filter.Rvec.copyTo(ToolR_pre.Rvec);
                ToolR_pre.ssize = ToolR.ssize;

                if (DrawTool)
                    CvDrawingUtils::draw3dAxis(frameL_copy, ToolR, StereoCameraParametersL);
                //cout << "ToolR: " << ToolR.Tvec << endl;
            }
            else if(!bool_initKFR)
            {
                if (DrawTool)
                    CvDrawingUtils::draw3dAxis(frameL_copy, ToolR, StereoCameraParametersL);
            }

//            cout << "Tool position: " << ToolL.Tvec.ptr<float>(0)[0] << " "
//                 << ToolL.Tvec.ptr<float>(0)[1] << " " << ToolL.Tvec.ptr<float>(0)[2] << endl;

            //Detect fabric ---------------------------------------------
            /*
             *
             *
             *
             *
             * ======================================================= */

            vector<Marker> fabricMarkers;
            fabricMarkers.resize(fabric_N);

            for(int i=0; i<fabric_N; i++)
            {
                findMarkerPose_byID(fabricMarkers[i], fabricID[i]);
                cout << "Marker " << fabricID[i] << " " << fabricMarkers[i].Tvec.ptr<float>(0)[0] << " "
                     << fabricMarkers[i].Tvec.ptr<float>(0)[1] << " " << fabricMarkers[i].Tvec.ptr<float>(0)[2] << endl;
            }


            // Write to trajectory file
            if (bool_write && (!bool_initKFL || !bool_initKFR || !bool_initKFM) )
            {
              myfile << Mandrel.Tvec.ptr<float>(0)[0] << " " << Mandrel.Tvec.ptr<float>(0)[1] << " " << Mandrel.Tvec.ptr<float>(0)[2] << " "
                     << Mandrel.Rvec.ptr<float>(0)[0] << " " << Mandrel.Rvec.ptr<float>(0)[1] << " " << Mandrel.Rvec.ptr<float>(0)[2] << " "
                     << ToolL.Tvec.ptr<float>(0)[0] << " " << ToolL.Tvec.ptr<float>(0)[1] << " " << ToolL.Tvec.ptr<float>(0)[2] << " "
                     << ToolL.Rvec.ptr<float>(0)[0] << " " << ToolL.Rvec.ptr<float>(0)[1] << " " << ToolL.Rvec.ptr<float>(0)[2] << " "
                     << ToolR.Tvec.ptr<float>(0)[0] << " " << ToolR.Tvec.ptr<float>(0)[1] << " " << ToolR.Tvec.ptr<float>(0)[2] << " "
                     << ToolR.Rvec.ptr<float>(0)[0] << " " << ToolR.Rvec.ptr<float>(0)[1] << " " << ToolR.Rvec.ptr<float>(0)[2] << " ";
            }

            if (bool_write)
            {
                for (int i=0; i<fabric_N; i++)
                {
                    myfile << fabricMarkers[i].Tvec.ptr<float>(0)[0] << " "
                           << fabricMarkers[i].Tvec.ptr<float>(0)[1] << " "
                           << fabricMarkers[i].Tvec.ptr<float>(0)[2] << " "
                           << fabricMarkers[i].Rvec.ptr<float>(0)[0] << " "
                           << fabricMarkers[i].Rvec.ptr<float>(0)[1] << " "
                           << fabricMarkers[i].Rvec.ptr<float>(0)[2] << " ";
                }
            }
            myfile << endl;

						//publish tool pose
				    msg.toolLTwist.linear.x = ToolL.Tvec.ptr<float>(0)[0];
						msg.toolLTwist.linear.y = ToolL.Tvec.ptr<float>(0)[1];
						msg.toolLTwist.linear.z = ToolL.Tvec.ptr<float>(0)[2];
				    msg.toolLTwist.angular.x = ToolL.Rvec.ptr<float>(0)[0];
						msg.toolLTwist.angular.y = ToolL.Rvec.ptr<float>(0)[1];
						msg.toolLTwist.angular.z = ToolL.Rvec.ptr<float>(0)[2];


				    msg.toolRTwist.linear.x = ToolR.Tvec.ptr<float>(0)[0];
						msg.toolRTwist.linear.y = ToolR.Tvec.ptr<float>(0)[1];
						msg.toolRTwist.linear.z = ToolR.Tvec.ptr<float>(0)[2];
				    msg.toolRTwist.angular.x = ToolR.Rvec.ptr<float>(0)[0];
						msg.toolRTwist.angular.y = ToolR.Rvec.ptr<float>(0)[1];
						msg.toolRTwist.angular.z = ToolR.Rvec.ptr<float>(0)[2];

				    msg.mandrelTwist.linear.x = Mandrel.Tvec.ptr<float>(0)[0];
						msg.mandrelTwist.linear.y = Mandrel.Tvec.ptr<float>(0)[1];
						msg.mandrelTwist.linear.z = Mandrel.Tvec.ptr<float>(0)[2];
				    msg.mandrelTwist.angular.x = Mandrel.Rvec.ptr<float>(0)[0];
						msg.mandrelTwist.angular.y = Mandrel.Rvec.ptr<float>(0)[1];
						msg.mandrelTwist.angular.z = Mandrel.Rvec.ptr<float>(0)[2];

						pub.publish(msg);
						rate.sleep();

            // Draw ROI -----------------------------------
            // Project fabric marker conners to 2D images

            vector<Point2f> imagePnt;
            vector<Point3f> pnt3_fabricmarker;

            imagePnt.resize(fabric_N);
            pnt3_fabricmarker.resize(fabric_N);

            Mat P1_ = P1(cv::Rect(0,0,3,3));
            Mat t_vec1 = Mat(3,1, CV_64F, double(0));
            Mat r_vec1 = Mat(3,1, CV_64F, double(0));

            for (int i=0; i<fabric_N; i++)
            {
                pnt3_fabricmarker[i].x = fabricMarkers[i].Tvec.ptr<float>(0)[0];
                pnt3_fabricmarker[i].y = fabricMarkers[i].Tvec.ptr<float>(0)[1];
                pnt3_fabricmarker[i].z = fabricMarkers[i].Tvec.ptr<float>(0)[2];
            }
            cv::projectPoints(pnt3_fabricmarker, r_vec1, t_vec1, P1_, D1, imagePnt);

//            cv::line(frameL_copy,imagePnt[0], imagePnt[1], Scalar(0,255,255));
//            cv::line(frameL_copy,imagePnt[1], imagePnt[3], Scalar(0,255,255));
//            cv::line(frameL_copy,imagePnt[3], imagePnt[2], Scalar(0,255,255));
//            cv::line(frameL_copy,imagePnt[2], imagePnt[0], Scalar(0,255,255));

            // -----------------------------


            //show input with augmented information and  the thresholded image
            usleep(100000);
            cv::imshow("left",frameL_copy);
            cv::imshow("right",frameR_copy);
            //cv::imshow("thres",MDetector.getThresholdedImage());

            key=cv::waitKey(waitTime);//wait for key to be pressed

            if(int('r') == char(key) || int('1') == char(key))
            {
                if (int('1') == char(key))
                    bool_write_raw = true;
                else
                    bool_write = true;

                time_t t = time(0);   // get time now
                struct tm * now = localtime( & t );
                char buffer [80];
                strftime (buffer,80,"%Y-%m-%d-%H-%M",now);

                // write a video
                string strTemp;
                strTemp = "outL_" + string(buffer) + ".avi";
                videoL.open(strTemp.c_str(),CV_FOURCC('M','J','P','G'),10, Size(frameL.size().width, frameL.size().height),true);
                strTemp = "outR_" + string(buffer) + ".avi";
                videoR.open(strTemp.c_str(),CV_FOURCC('M','J','P','G'),10, Size(frameR.size().width, frameR.size().height),true);
                strTemp = "toolmandrel_" + string(buffer) + ".txt";
                myfile.open(strTemp.c_str());

                if (!videoL.isOpened() || !videoR.isOpened())
                {
                    std::cout << "!!! Output video could not be opened" << std::endl;
                    return -1;
                }
                cout << "Start recording " << strTemp <<endl;


                if (int('r') == char(key))
                {
                    string fname_l = "outL_" + string(buffer) + ".png";
                    string fname_r = "outR_" + string(buffer) + ".png";

                    imwrite(fname_l.c_str(), frameL);
                    imwrite(fname_r.c_str(), frameR);
                }

                index = 1;
            }

            if (int('s') == char(key) || int('2') == char(key) )
            {
                bool_write = false;
                videoL.release();
                videoR.release();
                cout << "Stop recording!"<<endl;
            }

            if(bool_write)
            {
                videoL.write(frameL_copy);
                videoR.write(frameR_copy);
            }

            if(bool_write_raw)
            {
                videoL.write(frameL);
                videoR.write(frameR);
            }

            float dist1 = sqrt((ToolR.Tvec.ptr<float>(0)[0]-ToolL.Tvec.ptr<float>(0)[0])*(ToolR.Tvec.ptr<float>(0)[0]-ToolL.Tvec.ptr<float>(0)[0])+
                              (ToolR.Tvec.ptr<float>(0)[1]-ToolL.Tvec.ptr<float>(0)[1])*(ToolR.Tvec.ptr<float>(0)[1]-ToolL.Tvec.ptr<float>(0)[1])+
                              (ToolR.Tvec.ptr<float>(0)[2]-ToolL.Tvec.ptr<float>(0)[2])*(ToolR.Tvec.ptr<float>(0)[2]-ToolL.Tvec.ptr<float>(0)[2])
                        );
            float dist2 = sqrt((Mandrel.Tvec.ptr<float>(0)[0]-ToolL.Tvec.ptr<float>(0)[0])*(Mandrel.Tvec.ptr<float>(0)[0]-ToolL.Tvec.ptr<float>(0)[0])+
                              (Mandrel.Tvec.ptr<float>(0)[1]-ToolL.Tvec.ptr<float>(0)[1])*(Mandrel.Tvec.ptr<float>(0)[1]-ToolL.Tvec.ptr<float>(0)[1])+
                              (Mandrel.Tvec.ptr<float>(0)[2]-ToolL.Tvec.ptr<float>(0)[2])*(Mandrel.Tvec.ptr<float>(0)[2]-ToolL.Tvec.ptr<float>(0)[2])
                        );
            float high1 = (Mandrel.Tvec.ptr<float>(0)[2]-ToolL.Tvec.ptr<float>(0)[2]);
//            cout << "ToolR to ToolL distance: " << dist1 << endl;
////            cout << "Mandrel to ToolL distance: " << dist2 << endl;
////            cout << "Mandrel to ToolL high: " << high1 << endl;
//            //cout << "Mandrel: " << Mandrel.Tvec << endl;
////            //cout << "ToolR.Tvec: " << ToolR.Tvec << endl;
//            cout << "ToolL: " << ToolL.Tvec << endl;
//            cout << "ToolR: " << ToolR.Tvec << endl;
        }


        myfile.close();
    } catch (std::exception &ex)

    {
        cout<<"Exception :"<<ex.what()<<endl;
    }

}
/************************************
 *
 *
 *
 *
 ************************************/

void cvTackBarEvents(int pos,void*)
{
    if (iThresParam1<3) iThresParam1=3;
    if (iThresParam1%2!=1) iThresParam1++;
    if (ThresParam2<1) ThresParam2=1;
    ThresParam1=iThresParam1;
    ThresParam2=iThresParam2;
    MDetector.setThresholdParams(ThresParam1,ThresParam2);
//recompute
    MDetector.detect(TheInputImage,TheMarkers,TheCameraParameters);
    TheInputImage.copyTo(TheInputImageCopy);
    for (unsigned int i=0;i<TheMarkers.size();i++)	TheMarkers[i].draw(TheInputImageCopy,Scalar(0,0,255),1);
    //print other rectangles that contains no valid markers
    /*for (unsigned int i=0;i<MDetector.getCandidates().size();i++) {
        aruco::Marker m( MDetector.getCandidates()[i],999);
        m.draw(TheInputImageCopy,cv::Scalar(255,0,0));
    }*/

//draw a 3d cube in each marker if there is 3d info
    if (TheCameraParameters.isValid())
        for (unsigned int i=0;i<TheMarkers.size();i++)
            CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);

    cv::imshow("in",TheInputImageCopy);
    cv::imshow("thres",MDetector.getThresholdedImage());
}

/************************************
 *
 *
 *
 *
 ************************************/
bool readArguments ( int argc,char **argv )
{
    if (argc<5) {
        cerr<<"Invalid number of arguments"<<endl;
        cerr<<"Usage: [cameraL index] [intrinsicsL.yml] [cameraR index] [intrinsicsR.yml] [marker size] [0:camera 1:video] [(if video) left video] [(if video) right video]"<<endl;
        return false;
    }
    CameraL_ind=int(atof(argv[1]));
    theIntrinsicFileL=argv[2];
    CameraR_ind=int(atof(argv[3]));
    theIntrinsicFileR=argv[4];
    TheMarkerSize=atof(argv[5]);
    if (argc == 9)
    {
        streamMode=int(atof(argv[6]));
        videoFnameL = argv[7];
        videoFnameR = argv[8];
    }
    else
    {
        streamMode=0;
    }

    // Define marker corner model (square)
    markercornerLocal.resize(4);
    double halfSize=TheMarkerSize/2.;

   markercornerLocal[0].x = -halfSize;
   markercornerLocal[0].y = -halfSize;
   markercornerLocal[0].z = 0;
   markercornerLocal[1].x = -halfSize;
   markercornerLocal[1].y = halfSize;
   markercornerLocal[1].z = 0;
   markercornerLocal[2].x = halfSize;
   markercornerLocal[2].y = halfSize;
   markercornerLocal[2].z = 0;
   markercornerLocal[3].x = halfSize;
   markercornerLocal[3].y = -halfSize;
   markercornerLocal[3].z = 0;





    // Load stereo camera parameters
    string fileDir= "/home/charlie/Documents/workspace/ros_ws/CameraCalibration/";
    FileStorage f_Q(fileDir+ "Q.xml", FileStorage::READ); f_Q["Q"]>>Q;
    FileStorage f_mx1(fileDir+ "mx1.xml", FileStorage::READ); f_mx1["mx1"]>>mx1;
    FileStorage f_my1(fileDir+ "my1.xml", FileStorage::READ); f_my1["my1"]>>my1;
    FileStorage f_mx2(fileDir+ "mx2.xml", FileStorage::READ); f_mx2["mx2"]>>mx2;
    FileStorage f_my2(fileDir+ "my2.xml", FileStorage::READ); f_my2["my2"]>>my2;
    FileStorage f_P1(fileDir+ "P1.xml", FileStorage::READ); f_P1["P1"]>>P1;
    FileStorage f_P2(fileDir+ "P2.xml", FileStorage::READ); f_P2["P2"]>>P2;
    FileStorage f_D1(fileDir+ "D1.xml", FileStorage::READ); f_P1["D1"]>>D1;
    FileStorage f_D2(fileDir+ "D2.xml", FileStorage::READ); f_P2["D2"]>>D2;
    FileStorage f_T(fileDir+ "T.xml", FileStorage::READ); f_T["T"]>>T;
    FileStorage f_R(fileDir+ "R.xml", FileStorage::READ); f_R["R"]>>R;


    //read camera parameters
    TheCameraParametersL.readFromXMLFile(fileDir+theIntrinsicFileL);
    //TheCameraParametersL.resize(frameL.size());
    TheCameraParametersR.readFromXMLFile(fileDir+theIntrinsicFileR);
    //TheCameraParametersR.resize(frameR.size());

    StereoCameraParametersL.readFromXMLFile(fileDir+theIntrinsicFileL);
    //StereoCameraParametersL.resize(frameL.size());
    StereoCameraParametersR.readFromXMLFile(fileDir+theIntrinsicFileR);
    //StereoCameraParametersR.resize(frameR.size());

    return true;
}
/************************************
 *
 *
 *
 *
 ************************************/
void tuneFocus()
{
    // turn of autofocus -----------------------------------------------
    struct v4l2_control ctrl_auto, ctrl_absolute;
    ctrl_auto.id = V4L2_CID_FOCUS_AUTO;
    ctrl_auto.value = 0;
    ctrl_absolute.id = V4L2_CID_FOCUS_ABSOLUTE;
    ctrl_absolute.value = FOCUSABSOLUTE;
    int camera_dev, ret;

    if ((camera_dev = open(CAMERA0_DEV, O_RDONLY)) == -1)
    {
        cout << "couldn't open left camera"<<endl;
    }
    ret = ioctl(camera_dev, VIDIOC_S_CTRL, &ctrl_auto);
    ret = ioctl(camera_dev, VIDIOC_S_CTRL, &ctrl_absolute);

    if ((camera_dev = open(CAMERA1_DEV, O_RDONLY)) == -1)
    {
        cout << "couldn't open right camera"<<endl;
    }
    ret = ioctl(camera_dev, VIDIOC_S_CTRL, &ctrl_auto);

//    //Two different cameras----
//    ctrl_absolute.value = FOCUSABSOLUTE+5;
//    //-------------------------

    ret = ioctl(camera_dev, VIDIOC_S_CTRL, &ctrl_absolute);
    cout << "Focus length is fixed to " << FOCUSABSOLUTE << " !" <<endl;
}


/************************************
 *
 *
 *
 *
 ************************************/

void findStereoMarkers()
{
    // Search for markers that appear at both cameras
    StereoMarkersL.clear();
    StereoMarkersR.clear();

//    if (TheMarkersL.size() < TheMarkersR.size())
    {
        int searchInd = 0;
        for (unsigned int i=0; i<TheMarkersL.size();i++)
        {
            for (unsigned int j=searchInd; j<TheMarkersR.size(); j++)
            {
                if (TheMarkersL[i].id == TheMarkersR[j].id)
                {
                    StereoMarkersL.push_back(TheMarkersL[i]);
                    StereoMarkersR.push_back(TheMarkersR[j]);
                    searchInd = j+1;
                    break;
                }

            }
        }
    }
}

void findStereoMarkers_tracker(vector<Marker> markersLL, vector<bool> markersLL_bool,
                               vector<Marker> markersRL, vector<bool> markersRL_bool)
{
    // Search for markers that appear at both cameras
    StereoMarkersL.clear();
    StereoMarkersR.clear();

    int searchInd = 0;
    for (unsigned int i=0; i<TheMarkersL.size();i++)
    {
        for (unsigned int j=searchInd; j<TheMarkersR.size(); j++)
        {
            if (TheMarkersL[i].id == TheMarkersR[j].id)
            {
                StereoMarkersL.push_back(TheMarkersL[i]);
                StereoMarkersR.push_back(TheMarkersR[j]);
                searchInd = j+1;
                break;
            }

        }
    }

    for (unsigned int i=0; i<tool_N;i++)
    {
        if (markersLL_bool[i] && markersRL_bool[i])
        {
            StereoMarkersL.push_back(markersLL[i]);
            StereoMarkersR.push_back(markersRL[i]);
        }
    }

}

void estimateStereoMarkerPoses()
{
    vector<Point3f> marker4corners;
    Point3f cornerIncam;
    Point2f cornerL, cornerR;
    Mat point4D;
    Mat markerInCameraTrans;

    Mat P1_ = P1(cv::Rect(0,0,3,3));

    Mat t_vec1 = Mat(3,1, CV_64F, double(0));
    Mat r_vec1 = Mat(3,1, CV_64F, double(0));

    for (unsigned int i=0; i<StereoMarkersL.size(); i++)
    {
//        cout << "2D: " << i << " " << StereoMarkersL[i] << endl;
        //cout << "StereoMarkersR " << i << " " << StereoMarkersR[i] << endl;

        marker4corners.clear();
        for (int j=0; j<4; j++)
        {
            // Computer the corners' 3D location in sereo camera frame
            cornerL.x = StereoMarkersL[i][j].x;
            cornerL.y = StereoMarkersL[i][j].y;
            cornerR.x = StereoMarkersR[i][j].x;
            cornerR.y = StereoMarkersR[i][j].y;

            triangulatePoints(P1, P2, Mat(cornerL), Mat(cornerR), point4D);

            cornerIncam.x = point4D.at<float>(0,0)/point4D.at<float>(0,3);
            cornerIncam.y = point4D.at<float>(0,1)/point4D.at<float>(0,3);
            cornerIncam.z = point4D.at<float>(0,2)/point4D.at<float>(0,3);

//            cout<< "cornerIncam: " << cornerIncam << endl;
            marker4corners.push_back(cornerIncam);

        }

        // Reproject 3D points to left image
        vector<Point2f> imagePnt;
        cv::projectPoints(marker4corners, r_vec1, t_vec1, P1_, D1, imagePnt);
        float size_ = 0;
        float temp_x = 0, temp_y = 0, temp_z = 0;
        for (int j=0; j<4; j++)
        {
            StereoMarkersL[i][j].x = imagePnt[j].x;
            StereoMarkersL[i][j].y = imagePnt[j].y;
            size_ = size_ + norm(Mat(marker4corners[j%4]), Mat(marker4corners[(j+1)%4]));
            temp_x += marker4corners[j].x/4;
            temp_y += marker4corners[j].y/4;
            temp_z += marker4corners[j].z/4;
        }



        StereoMarkersL[i].ssize = size_/4;
        double halfSize=StereoMarkersL[i].ssize/2.;
           markercornerLocal[0].x = -halfSize;
           markercornerLocal[0].y = -halfSize;
           markercornerLocal[0].z = 0;
           markercornerLocal[1].x = -halfSize;
           markercornerLocal[1].y = halfSize;
           markercornerLocal[1].z = 0;
           markercornerLocal[2].x = halfSize;
           markercornerLocal[2].y = halfSize;
           markercornerLocal[2].z = 0;
           markercornerLocal[3].x = halfSize;
           markercornerLocal[3].y = -halfSize;
           markercornerLocal[3].z = 0;

//        cout << "size: " << StereoMarkersL[i].ssize << endl;

        // Estimate marker's 3D pose by its corners' 3D locations
        find_rigidTransformation(markercornerLocal,marker4corners,markerInCameraTrans);

        // Why need to rotate ????????--------------------------
        Mat tranX90 = (Mat_<double>(4,4) << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1);
        markerInCameraTrans = markerInCameraTrans * tranX90;


        StereoMarkersL[i].Tvec.ptr<float>(0)[0] = markerInCameraTrans.at<double>(0,3);
        StereoMarkersL[i].Tvec.ptr<float>(0)[1] = markerInCameraTrans.at<double>(1,3);
        StereoMarkersL[i].Tvec.ptr<float>(0)[2] = markerInCameraTrans.at<double>(2,3);

        Mat rotationMat = markerInCameraTrans.colRange(0, 3).rowRange(0, 3);
        //Mat rotX90 = (Mat_<double>(3,3) << 1, 0, 0, 0, 0, -1, 0, 1, 0);
        //rotationMat = rotationMat * rotX90;

        // Correct pose from left
        cv::Mat rvec;
        cv::Rodrigues(rotationMat, rvec);

        StereoMarkersL[i].Tvec.convertTo(StereoMarkersL[i].Tvec, CV_32F);
        rvec.convertTo(StereoMarkersL[i].Rvec, CV_32F);

        // Correct pose from right

        // Transfer to right camera frame

    }
}




/************************************
 *
 *
 *
 *kalmanFilterTrans
 ************************************/

void initializeKF(KalmanFilter &KF, KalmanFilter &KF_rot)
{

    KF.transitionMatrix = (Mat_<float>(6,6)
            <<1,0,0,10,0,0,  0,1,0,0,10,0, 0,0,1,0,0,10, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1);
    KF.statePre.at<float>(3) = 0;
    KF.statePre.at<float>(4) = 0;
    KF.statePre.at<float>(5) = 0;
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-4));//e-4));
    setIdentity(KF.measurementNoiseCov, Scalar::all(10));//10
    setIdentity(KF.errorCovPost, Scalar::all(.1));

    //Mat_<float> measurement(6,1); measurement.setTo(Scalar(0));


    KF_rot.transitionMatrix = (Mat_<float>(6,6)
            <<1,0,0,10,0,0,  0,1,0,0,10,0, 0,0,1,0,0,10, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1);
    KF_rot.statePre.at<float>(3) = 0;
    KF_rot.statePre.at<float>(4) = 0;
    KF_rot.statePre.at<float>(5) = 0;
    setIdentity(KF_rot.measurementMatrix);
    setIdentity(KF_rot.processNoiseCov, Scalar::all(1));//e-4));MandrelDetected
    setIdentity(KF_rot.measurementNoiseCov, Scalar::all(10));//10
    setIdentity(KF_rot.errorCovPost, Scalar::all(.1));
}


void kalmanFilterPose(Marker &pose_detected, Marker &pose_previous, Marker &pose_filtered,
                      KalmanFilter &KF, KalmanFilter &KF_rot, bool &bool_initKF)
{

    // Initialise KF -----------
    if (bool_initKF)
    {
        bool_initKF = false;

        KF.statePre.at<float>(0) = pose_detected.Tvec.ptr<float>(0)[0];
        KF.statePre.at<float>(1) = pose_detected.Tvec.ptr<float>(0)[1];
        KF.statePre.at<float>(2) = pose_detected.Tvec.ptr<float>(0)[2];

        KF_rot.statePre.at<float>(0) = pose_detected.Rvec.ptr<float>(0)[0];
        KF_rot.statePre.at<float>(1) = pose_detected.Rvec.ptr<float>(0)[1];
        KF_rot.statePre.at<float>(2) = pose_detected.Rvec.ptr<float>(0)[2];
    }

    // Prediction by KF
    Mat prediction = KF.predict();
    Mat prediction_rot = KF_rot.predict();

    // If tool detected
    if (pose_detected.Rvec.ptr<float>(0)[0] != -1000)
    {
        float theta = norm(pose_detected.Rvec);

        if ((pose_previous.Rvec.dot(pose_detected.Rvec)) < 0 || theta > 5)
        {
            pose_detected.Rvec.ptr<float>(0)[0] = (1-3.14*2/theta) * pose_detected.Rvec.ptr<float>(0)[0];
            pose_detected.Rvec.ptr<float>(0)[1] = (1-3.14*2/theta) * pose_detected.Rvec.ptr<float>(0)[1];
            pose_detected.Rvec.ptr<float>(0)[2] = (1-3.14*2/theta) * pose_detected.Rvec.ptr<float>(0)[2];
            //cout << " reverse! ----" << endl;
        }

        Mat estimated = KF.correct(pose_detected.Tvec);
        pose_filtered.Tvec.ptr<float>(0)[0] = estimated.at<float>(0);
        pose_filtered.Tvec.ptr<float>(0)[1] = estimated.at<float>(1);
        pose_filtered.Tvec.ptr<float>(0)[2] = estimated.at<float>(2);

        Mat estimated_rot = KF_rot.correct(pose_detected.Rvec);
        pose_filtered.Rvec.ptr<float>(0)[0] = estimated_rot.at<float>(0);
        pose_filtered.Rvec.ptr<float>(0)[1] = estimated_rot.at<float>(1);
        pose_filtered.Rvec.ptr<float>(0)[2] = estimated_rot.at<float>(2);

        pose_filtered.ssize = pose_detected.ssize;
    }

    // If tool lost -------
    else if(!bool_initKF)
    {
        pose_previous.Tvec.copyTo(pose_detected.Tvec);
        pose_previous.Rvec.copyTo(pose_detected.Rvec);

        float theta = norm(pose_detected.Rvec);

        pose_filtered.Tvec.ptr<float>(0)[0] = prediction.at<float>(0);
        pose_filtered.Tvec.ptr<float>(0)[1] = prediction.at<float>(1);
        pose_filtered.Tvec.ptr<float>(0)[2] = prediction.at<float>(2);

        pose_filtered.Rvec.ptr<float>(0)[0] = prediction_rot.at<float>(0);
        pose_filtered.Rvec.ptr<float>(0)[1] = prediction_rot.at<float>(1);
        pose_filtered.Rvec.ptr<float>(0)[2] = prediction_rot.at<float>(2);

        if ((pose_previous.Rvec.dot(pose_filtered.Rvec)) < 0)
        {
            pose_filtered.Rvec.ptr<float>(0)[0] = (1-3.14*2/theta) * pose_filtered.Rvec.ptr<float>(0)[0];
            pose_filtered.Rvec.ptr<float>(0)[1] = (1-3.14*2/theta) * pose_filtered.Rvec.ptr<float>(0)[1];
            pose_filtered.Rvec.ptr<float>(0)[2] = (1-3.14*2/theta) * pose_filtered.Rvec.ptr<float>(0)[2];
            cout << " no detection! reverse! ----" << endl;
        }

        pose_filtered.ssize = pose_previous.ssize;
    }



}

/************************************
 *
 *
 *
 *estimateToolPose
 ************************************/

bool estimateToolPose(Marker &Tool, const int toolID[], const int N, const float tsize)
{
    bool toolDetected = false;

    Mat markerInCameraTrans = Mat::eye(4,4,CV_32F),
        toolInMarkerTrans= Mat::eye(4,4,CV_32F),
        toolInCameraTrans = Mat::eye(4,4,CV_32F);

    //int N = (sizeof(toolID)/sizeof(toolID[2]));
    double rotXangle;
    int ind;
    for (int i=0; i<N; i++)
    {
        for (int j=0; j<StereoMarkersL.size(); j++)
        {
            if (StereoMarkersL[j].id == toolID[i])
            {
                ToolMarkersL.push_back(StereoMarkersL[j]);
                ind = j;
                rotXangle = 2*M_PI / N * i;
                toolDetected = true;
                break;
            }
        }
    }

    if (toolDetected)
    {
        markerInCameraTrans.at<float>(0,3) = StereoMarkersL[ind].Tvec.ptr<float>(0)[0];
        markerInCameraTrans.at<float>(1,3) = StereoMarkersL[ind].Tvec.ptr<float>(0)[1];
        markerInCameraTrans.at<float>(2,3) = StereoMarkersL[ind].Tvec.ptr<float>(0)[2];

        cv::Rodrigues(StereoMarkersL[ind].Rvec, markerInCameraTrans.colRange(0,3).rowRange(0,3));

        toolInMarkerTrans.at<float>(1,1) = cos(rotXangle); toolInMarkerTrans.at<float>(1,2) = -sin(rotXangle);
        toolInMarkerTrans.at<float>(2,1) = sin(rotXangle); toolInMarkerTrans.at<float>(2,2) =  cos(rotXangle);
        // Shift along y axis
        //toolInMarkerTrans.at<float>(1,3) =  - StereoMarkersL[ind].ssize/(2*tan(M_PI/N));
        toolInMarkerTrans.at<float>(1,3) =  - tsize/(2*tan(M_PI/N));

        // Computer tool pose
        toolInCameraTrans = markerInCameraTrans * toolInMarkerTrans;


        Tool.Tvec.ptr<float>(0)[0] = toolInCameraTrans.at<float>(0,3);
        Tool.Tvec.ptr<float>(0)[1] = toolInCameraTrans.at<float>(1,3);
        Tool.Tvec.ptr<float>(0)[2] = toolInCameraTrans.at<float>(2,3);

        // Correct pose for left
        cv::Mat rvec;
        cv::Rodrigues(toolInCameraTrans.colRange(0,3).rowRange(0,3), rvec);

        Tool.Tvec.convertTo(Tool.Tvec, CV_32F);
        rvec.convertTo(Tool.Rvec, CV_32F);
        Tool.ssize = StereoMarkersL[ind].ssize;

//        cout << "Tool position: " << Tool.Tvec.ptr<float>(0)[0] << " "
//             << Tool.Tvec.ptr<float>(0)[1] << " " << Tool.Tvec.ptr<float>(0)[2] << endl;
        return true;
    }
    else
    {
        Tool.Tvec.ptr<float>(0)[0] = -1000;
        Tool.Tvec.ptr<float>(0)[1] = -1000;
        Tool.Tvec.ptr<float>(0)[2] = -1000;
        Tool.Rvec.ptr<float>(0)[0] = -1000;
        Tool.Rvec.ptr<float>(0)[1] = -1000;
        Tool.Rvec.ptr<float>(0)[2] = -1000;
        return false;
    }

}

bool findMarkerPose_byID(Marker &fabric, int id)
{
    fabric.id = id;
    bool foundFabricMarker = false;

    for (int i=0; i<StereoMarkersL.size(); i++)
    {
        if (StereoMarkersL[i].id == id)
        {
            foundFabricMarker = true;
            for (int j=0; j<3; j++)
            {
                fabric.Tvec.ptr<float>(0)[j] = StereoMarkersL[i].Tvec.ptr<float>(0)[j];
                fabric.Rvec.ptr<float>(0)[j] = StereoMarkersL[i].Rvec.ptr<float>(0)[j];
            }
        }
    }

    if (foundFabricMarker == false)
    {
        for (int j=0; j<3; j++)
        {
            fabric.Tvec.ptr<float>(0)[j] = -1000;
            fabric.Rvec.ptr<float>(0)[j] = -1000;
        }
    }
}


bool estimateToolPose_multimarkers(Marker &Tool, const int toolID[], const int N, const float tsize)
{
    bool toolDetected = false;

    Mat markerInCameraTrans = Mat::eye(4,4,CV_32F),
        toolInMarkerTrans= Mat::eye(4,4,CV_32F),
        toolInCameraTrans = Mat::eye(4,4,CV_32F);

    //int N = (sizeof(toolID)/sizeof(toolID[2]));
    double rotXangle;
    int ind;
    vector<cv::Mat> toolMarkersTrans;

    for (int i=0; i<N; i++)
    {
        //cout << i << " " << toolID[i] << " " << N << endl;
        for (int j=0; j<StereoMarkersL.size(); j++)
        {
            if (StereoMarkersL[j].id == toolID[i])
            {
                //ToolMarkersL.push_back(StereoMarkersL[j]);
                ind = j;
                rotXangle = 2*M_PI / N * i;
                toolDetected = true;
                //break;

                // Compute tool pose by marker pose
                markerInCameraTrans.at<float>(0,3) = StereoMarkersL[ind].Tvec.ptr<float>(0)[0];
                markerInCameraTrans.at<float>(1,3) = StereoMarkersL[ind].Tvec.ptr<float>(0)[1];
                markerInCameraTrans.at<float>(2,3) = StereoMarkersL[ind].Tvec.ptr<float>(0)[2];

                cv::Rodrigues(StereoMarkersL[ind].Rvec, markerInCameraTrans.colRange(0,3).rowRange(0,3));

                toolInMarkerTrans.at<float>(1,1) = cos(rotXangle); toolInMarkerTrans.at<float>(1,2) = -sin(rotXangle);
                toolInMarkerTrans.at<float>(2,1) = sin(rotXangle); toolInMarkerTrans.at<float>(2,2) =  cos(rotXangle);
                // Shift along y axis
                //toolInMarkerTrans.at<float>(1,3) =  - StereoMarkersL[ind].ssize/(2*tan(M_PI/N));
                toolInMarkerTrans.at<float>(1,3) =  - tsize/(2*tan(M_PI/N));

                // Computer tool pose
                toolInCameraTrans = markerInCameraTrans * toolInMarkerTrans;
                toolMarkersTrans.push_back(toolInCameraTrans);
            }
        }
    }

    // Compute tool pose by two markers ------------------------------
    if (toolMarkersTrans.size()>1)
    {
        // Compute average of two transform matrixes
        toolInCameraTrans.at<float>(0,3) = 0.5* (toolInCameraTrans.at<float>(0,3) + toolMarkersTrans[0].at<float>(0,3));
        toolInCameraTrans.at<float>(1,3) = 0.5* (toolInCameraTrans.at<float>(1,3) + toolMarkersTrans[0].at<float>(1,3));
        toolInCameraTrans.at<float>(2,3) = 0.5* (toolInCameraTrans.at<float>(2,3) + toolMarkersTrans[0].at<float>(2,3));

        cv::Mat rvec_0, rvec_1;
        cv::Rodrigues(toolMarkersTrans[0].colRange(0,3).rowRange(0,3), rvec_0);
        cv::Rodrigues(toolInCameraTrans.colRange(0,3).rowRange(0,3), rvec_1);
        rvec_0.convertTo(rvec_0, CV_32F);
        rvec_1.convertTo(rvec_1, CV_32F);
        rvec_0.ptr<float>(0)[0] = 0.5 * (rvec_0.ptr<float>(0)[0] + rvec_1.ptr<float>(0)[0]);
        rvec_0.ptr<float>(0)[1] = 0.5 * (rvec_0.ptr<float>(0)[1] + rvec_1.ptr<float>(0)[1]);
        rvec_0.ptr<float>(0)[2] = 0.5 * (rvec_0.ptr<float>(0)[2] + rvec_1.ptr<float>(0)[2]);
        cv::Rodrigues(rvec_0, toolInCameraTrans.colRange(0,3).rowRange(0,3));
    }


    if (toolDetected == true)
    {
        Tool.Tvec.ptr<float>(0)[0] = toolInCameraTrans.at<float>(0,3);
        Tool.Tvec.ptr<float>(0)[1] = toolInCameraTrans.at<float>(1,3);
        Tool.Tvec.ptr<float>(0)[2] = toolInCameraTrans.at<float>(2,3);

        // Correct pose for left
        cv::Mat rvec;
        cv::Rodrigues(toolInCameraTrans.colRange(0,3).rowRange(0,3), rvec);

        Tool.Tvec.convertTo(Tool.Tvec, CV_32F);
        rvec.convertTo(Tool.Rvec, CV_32F);
        Tool.ssize = StereoMarkersL[ind].ssize;

        return true;
    }
    else
    {
        Tool.Tvec.ptr<float>(0)[0] = -1000;
        Tool.Tvec.ptr<float>(0)[1] = -1000;
        Tool.Tvec.ptr<float>(0)[2] = -1000;
        Tool.Rvec.ptr<float>(0)[0] = -1000;
        Tool.Rvec.ptr<float>(0)[1] = -1000;
        Tool.Rvec.ptr<float>(0)[2] = -1000;

        return false;
    }


}

bool estimateToolPose_singlemarker(aruco::Marker &tool, vector<int> toolID, int N, float markersize)
{
    bool toolDetected = false;
    vector<int> markerInd(2);
    Marker tool_est;
    vector<Marker> tool_all;

    for (int i=0; i<N; i++)
    {
        for (int j=0; j<StereoMarkersL.size(); j++)
        {
            if (StereoMarkersL[j].id == toolID[i])
            {
                tool.id = toolID[i];
                markerInd[0] = j;
                toolDetected = true;
                break;
            }
        }
    }

    if (toolDetected)
    {
        for (int i=0; i<3; i++)
        {
            tool.Tvec.ptr<float>(0)[i] = StereoMarkersL[markerInd[0]].Tvec.ptr<float>(0)[i];
            tool.Rvec.ptr<float>(0)[i] = StereoMarkersL[markerInd[0]].Rvec.ptr<float>(0)[i];
        }
//cout<<tool.Tvec.ptr<float>(0)[0]<<" "<<tool.Tvec.ptr<float>(0)[1]<<" "<<tool.Tvec.ptr<float>(0)[2]<<" "<<tool.Rvec.ptr<float>(0)[0]<<" "<<tool.Rvec.ptr<float>(0)[1]<<" "<<tool.Rvec.ptr<float>(0)[2]<<endl;

        tool.ssize = StereoMarkersL[markerInd[0]].ssize;
        return true;
    }
    //////// Use tracking ////////
    tool.Tvec.ptr<float>(0)[0] = -1000;
    tool.Tvec.ptr<float>(0)[1] = -1000;
    tool.Tvec.ptr<float>(0)[2] = -1000;
    tool.Rvec.ptr<float>(0)[0] = -1000;
    tool.Rvec.ptr<float>(0)[1] = -1000;
    tool.Rvec.ptr<float>(0)[2] = -1000;
    tool.ssize = -1000;
//cout<<tool.Tvec.ptr<float>(0)[0]<<" "<<tool.Tvec.ptr<float>(0)[1]<<" "<<tool.Tvec.ptr<float>(0)[2]<<" "<<tool.Rvec.ptr<float>(0)[0]<<" "<<tool.Rvec.ptr<float>(0)[1]<<" "<<tool.Rvec.ptr<float>(0)[2]<<endl;
    return false;

}


bool estimateMandrelPose(Marker &man, const int manID[], const int N, const float mRadius)
{
    bool manDetected = false;

    Mat markerInCameraTrans = Mat::eye(4,4,CV_32F),
        manInMarkerTrans= Mat::eye(4,4,CV_32F),
        manInCameraTrans = Mat::eye(4,4,CV_32F);

    //int N = (sizeof(toolID)/sizeof(toolID[2]));
    double rotZangle;
    int ind;
    for (int i=0; i<N; i++)
    {
        for (int j=0; j<StereoMarkersL.size(); j++)
        {
            if (StereoMarkersL[j].id == manID[i])
            {
                ind = j;
                rotZangle = 2*M_PI / N * i;
                manDetected = true;
                break;
            }
        }
    }

    if (manDetected)
    {
        markerInCameraTrans.at<float>(0,3) = StereoMarkersL[ind].Tvec.ptr<float>(0)[0];
        markerInCameraTrans.at<float>(1,3) = StereoMarkersL[ind].Tvec.ptr<float>(0)[1];
        markerInCameraTrans.at<float>(2,3) = StereoMarkersL[ind].Tvec.ptr<float>(0)[2];

        cv::Rodrigues(StereoMarkersL[ind].Rvec, markerInCameraTrans.colRange(0,3).rowRange(0,3));

//        manInMarkerTrans.at<float>(0,0) = cos(rotZangle); manInMarkerTrans.at<float>(0,1) = -sin(rotZangle);
//        manInMarkerTrans.at<float>(1,0) = sin(rotZangle); manInMarkerTrans.at<float>(1,1) =  cos(rotZangle);
        manInMarkerTrans.at<float>(1,1) = cos(rotZangle); manInMarkerTrans.at<float>(1,2) = -sin(rotZangle);
        manInMarkerTrans.at<float>(2,1) = sin(rotZangle); manInMarkerTrans.at<float>(2,2) =  cos(rotZangle);
        //         Shift along y axis
//        manInMarkerTrans.at<float>(1,3) =  - mRadius/2;
        manInMarkerTrans.at<float>(1,3) =  - mSize/(2*tan(M_PI/mandrel_N));

        // Compute tool pose
        manInCameraTrans = markerInCameraTrans * manInMarkerTrans;

        man.Tvec.ptr<float>(0)[0] = manInCameraTrans.at<float>(0,3);
        man.Tvec.ptr<float>(0)[1] = manInCameraTrans.at<float>(1,3);
        man.Tvec.ptr<float>(0)[2] = manInCameraTrans.at<float>(2,3);

        // Correct pose from left
        cv::Mat rvec;
        cv::Rodrigues(manInCameraTrans.colRange(0,3).rowRange(0,3), rvec);

        man.Tvec.convertTo(man.Tvec, CV_32F);
        rvec.convertTo(man.Rvec, CV_32F);
        man.ssize = StereoMarkersL[ind].ssize;

  //      cout << "Tool position: " << Tool.Tvec.ptr<float>(0)[0] << " "
    //         << Tool.Tvec.ptr<float>(0)[1] << " " << Tool.Tvec.ptr<float>(0)[2] << endl;
        return true;
    }
    else
    {
        man.Tvec.ptr<float>(0)[0] = -1000;
        man.Tvec.ptr<float>(0)[1] = -1000;
        man.Tvec.ptr<float>(0)[2] = -1000;
        man.Rvec.ptr<float>(0)[0] = -1000;
        man.Rvec.ptr<float>(0)[1] = -1000;
        man.Rvec.ptr<float>(0)[2] = -1000;
        return false;
    }

}




void drawNeedleFnt(cv::Mat3b frame, bool bool_frameL, cv::Mat ToolHNeedle, cv::Mat ToolHNeedleTip, int toolIndex)
{
    // TODO: for toolR !!!!!!!!!!!!!!
    // For frame right -------------------
    Mat r_vec2 = Mat(3,1, CV_32F, float(0));
    Mat P2_ = Mat(3,3, CV_32F, float(0));
    Mat clHcr, crHn, crHp;
    clHcr = Mat::eye(4,4,CV_32F); crHn = Mat::eye(4,4,CV_32F); crHp = Mat::eye(4,4,CV_32F);
    cv::Mat T_homogeneous(4,1,cv::DataType<float>::type); // translation vector
    cv::decomposeProjectionMatrix(P2, P2_, r_vec2, T_homogeneous);
    clHcr.at<float>(0,3) = (float)T_homogeneous.at<double>(0,0)/T_homogeneous.at<double>(3,0);
    clHcr.at<float>(1,3) = (float)T_homogeneous.at<double>(1,0)/T_homogeneous.at<double>(3,0);
    clHcr.at<float>(2,3) = (float)T_homogeneous.at<double>(2,0)/T_homogeneous.at<double>(3,0);
    P2_.copyTo(StereoCameraParametersR.CameraMatrix);

    cv::Mat tHn, cHt, cHn;
    tHn = Mat::eye(4,4,CV_32F);cHt = Mat::eye(4,4,CV_32F);cHn = Mat::eye(4,4,CV_32F);
    Marker Needle, NeedleEnd;

    // Compute needle tip
    ToolHNeedle.copyTo(tHn);
    tHn.convertTo(tHn, CV_32F);

    if (toolIndex == 1)
    {
        cHt.at<float>(0,3) = ToolL.Tvec.ptr<float>(0)[0];
        cHt.at<float>(1,3) = ToolL.Tvec.ptr<float>(0)[1];
        cHt.at<float>(2,3) = ToolL.Tvec.ptr<float>(0)[2];
        cv::Rodrigues(ToolL.Rvec, cHt.colRange(0,3).rowRange(0,3));
    }

    cHn = cHt * tHn;
    cv::Rodrigues(cHn.colRange(0,3).rowRange(0,3), Needle.Rvec);
    Needle.Tvec.ptr<float>(0)[0] = cHn.at<float>(0,3);
    Needle.Tvec.ptr<float>(0)[1] = cHn.at<float>(1,3);
    Needle.Tvec.ptr<float>(0)[2] = cHn.at<float>(2,3);
    Needle.ssize = 0.002;

    if (!bool_frameL)
    {
        crHn = clHcr.inv() * cHn;
        cv::Rodrigues(crHn.colRange(0,3).rowRange(0,3), Needle.Rvec);
        Needle.Tvec.ptr<float>(0)[0] = crHn.at<float>(0,3);
        Needle.Tvec.ptr<float>(0)[1] = crHn.at<float>(1,3);
        Needle.Tvec.ptr<float>(0)[2] = crHn.at<float>(2,3);
    }

    // Compute needle end ---------------------------------------
    ToolHNeedleTip.copyTo(tHn);
    tHn.convertTo(tHn, CV_32F);

    cHn = cHt * tHn;
    cv::Rodrigues(cHn.colRange(0,3).rowRange(0,3), NeedleEnd.Rvec);
    NeedleEnd.Tvec.ptr<float>(0)[0] = cHn.at<float>(0,3);
    NeedleEnd.Tvec.ptr<float>(0)[1] = cHn.at<float>(1,3);
    NeedleEnd.Tvec.ptr<float>(0)[2] = cHn.at<float>(2,3);
    NeedleEnd.ssize = 0.002;


    if (!bool_frameL)
    {
        crHn = clHcr.inv() * cHn;
        cv::Rodrigues(crHn.colRange(0,3).rowRange(0,3), NeedleEnd.Rvec);
        NeedleEnd.Tvec.ptr<float>(0)[0] = crHn.at<float>(0,3);
        NeedleEnd.Tvec.ptr<float>(0)[1] = crHn.at<float>(1,3);
        NeedleEnd.Tvec.ptr<float>(0)[2] = crHn.at<float>(2,3);
    }

    if (bool_frameL)
    {
        CvDrawingUtils::draw3dAxis(frame, Needle, StereoCameraParametersL);
        CvDrawingUtils::draw3dAxis(frame, NeedleEnd, StereoCameraParametersL);
    }
    else
    {
        CvDrawingUtils::draw3dAxis(frame, Needle, StereoCameraParametersR);
        CvDrawingUtils::draw3dAxis(frame, NeedleEnd, StereoCameraParametersR);
    }

}

/************************************
 *
 *
 *
 *
 ************************************/
template <typename CV_POINT3_SRC, typename CV_POINT3_DST>
static void find_rigidTransformation(const std::vector<CV_POINT3_SRC> &_src,
                                     const std::vector<CV_POINT3_DST> &_dst,
                                     cv::Mat &H_src2dst)
{
    assert(_src.size() == _dst.size());
    // get local copy of src and dst
    std::vector<CV_POINT3_SRC> src = _src;
    std::vector<CV_POINT3_DST> dst = _dst;
    // 1. Calculate centroid of each point cloud
    CV_POINT3_SRC centroid_A(0.0, 0.0, 0.0);
    CV_POINT3_DST centroid_B(0.0, 0.0, 0.0);

    for (int i = 0; i < src.size(); i++)
    {
        centroid_A += src[i];
        centroid_B += dst[i];
    }
    centroid_A.x /= src.size();
    centroid_A.y /= src.size();
    centroid_A.z /= src.size();
    centroid_B.x /= src.size();
    centroid_B.y /= src.size();
    centroid_B.z /= src.size();


    // 2. Center the points according to the centroid
    cv::Mat A(cv::Mat::zeros(src.size(), 3, CV_64FC1)), B(cv::Mat::zeros(dst.size(), 3, CV_64FC1));
    for (int i = 0; i < src.size(); i++)
    {
        src[i] -= centroid_A;
        dst[i] -= centroid_B;

        A.at<double>(i, 0) = src[i].x;
        A.at<double>(i, 1) = src[i].y;
        A.at<double>(i, 2) = src[i].z;
        B.at<double>(i, 0) = dst[i].x;
        B.at<double>(i, 1) = dst[i].y;
        B.at<double>(i, 2) = dst[i].z;
    }

    cv::Mat H, R, t;
    H = A.t() * B;

    cv::SVD svdobj(H,cv::SVD::FULL_UV);
    R = svdobj.vt.t() * svdobj.u.t();
    R = svdobj.vt.t() * svdobj.u.t();


    if(cv::determinant(R) < 0)
    {
        //std::cout << "Reflection detected\n";
        cv::Mat V = svdobj.vt.t();
        V.col(2) = -V.col(2);
        R = V * svdobj.u.t();
        //R.col(2) = -R.col(2);
    }
    cv::Mat mat_centroid_A, mat_centroid_B;
    mat_centroid_A = ( cv::Mat_<double>(1,3)<< centroid_A.x, centroid_A.y, centroid_A.z);
    mat_centroid_B = ( cv::Mat_<double>(1,3)<< centroid_B.x, centroid_B.y, centroid_B.z);
    t = -R * mat_centroid_A.t() + mat_centroid_B.t();

    H_src2dst = cv::Mat::eye(4, 4, CV_64FC1);
    cv::Mat R_temp = H_src2dst.colRange(0, 3).rowRange(0, 3);
    cv::Mat t_temp = H_src2dst.col(3).rowRange(0,3);
    R.copyTo(R_temp);
    t.copyTo(t_temp);
    // std::cout << R << std::endl;
    // std::cout << t << std::endl;
    // std::cout << H_src2dst << std::endl;
}

/************************************
 * Track Markers
 *
 *
 *
 ************************************/
vector<bool> trackMarkers(cv::Mat &track_image, vector<Marker> &detectedMarkers,
                          vector<Marker> &toolMarkers, vector<dvrk::TrackerGeneral> &trackers,
                          CameraParameters camParams ,float markerSizeMeters)
{
    vector<bool> markerDetected;
    for (int i=0; i<tool_N; i++)
    {
        markerDetected.push_back(false);
    }
    // Loop detected markers, fill in tool markers
    for (int i=0; i<toolMarkers.size(); i++)
    {
        for (int j=0; j<detectedMarkers.size(); j++)
        {
            if (toolMarkers[i].id == detectedMarkers[j].id)
            {
                toolMarkers[i].clear();
                toolMarkers[i].Tvec.ptr<float>(0)[0] = detectedMarkers[j].Tvec.ptr<float>(0)[0];
                toolMarkers[i].Tvec.ptr<float>(0)[1] = detectedMarkers[j].Tvec.ptr<float>(0)[1];
                toolMarkers[i].Tvec.ptr<float>(0)[2] = detectedMarkers[j].Tvec.ptr<float>(0)[2];
                toolMarkers[i].Rvec.ptr<float>(0)[0] = detectedMarkers[j].Rvec.ptr<float>(0)[0];
                toolMarkers[i].Rvec.ptr<float>(0)[1] = detectedMarkers[j].Rvec.ptr<float>(0)[1];
                toolMarkers[i].Rvec.ptr<float>(0)[2] = detectedMarkers[j].Rvec.ptr<float>(0)[2];

                for (int k=0; k<4; k++)
                {//Copy corners
                    toolMarkers[i].push_back(detectedMarkers[j][k]);
                }
                markerDetected[i]=true;
                break;
            }
        }

    }

    // initialise tracker


    // track if not detected
    // if tracker is initialized -------
    for (int i=0; i<trackers.size(); i++)
    {
        if (toolMarkers[i].Tvec.ptr<float>(0)[0] != -1000)
        {
            vector<cv::Point2f> corners;
            for (int k=0; k<4; k++)
            {
                corners.push_back(toolMarkers[i][k]);
            }
            bool before, after;
            before = markerDetected[i];
            if (BOOL_TRACK)
                markerDetected[i] = trackers[i].track(track_image, corners, markerDetected[i]);
            after = markerDetected[i];
            if(markerDetected[i])
            {
                for ( int c=0;c<4;c++ )
                    toolMarkers[i][c]=corners[c];

                toolMarkers[i].calculateExtrinsics(markerSizeMeters,camParams);
            }

            if (before == false && after == true)
            {
                //cout << toolMarkers[i].id << " is tracked " << endl;
            }
        }
    }

    return markerDetected;

}
