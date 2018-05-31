//#ifndef TRACKTOOLS
//#define TRACKTOOLS

//#endif // TRACKTOOLS

#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>
#include "mycvdrawingutils.h"
#include "mymarkerdetector.h"


#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/v4l2-mediabus.h>

namespace myaruco{

    class TrackTools
    {
    public:
        TrackTools(string fileDir);
//        void initTool(float markerSize, vector<int> toolid);
        void initTools(vector<float> markersSizes, vector<vector<int> > toolsids);

        bool findStereoMarkers();
        void estimateStereoMarkerPose();
        bool estimateToolPose(aruco::Marker &tool, vector<int> toolID, int N);
        void detectTools(cv::Mat3b frameL, cv::Mat3b frameR,
                         vector<cv::Mat> &toolsTvec,
                         vector<cv::Mat> &toolsRvec,
                         vector<bool> &detected);

        void drawStereoMarkers(cv::Mat3b frameL, cv::Mat3b frameR);
        void drawTools(cv::Mat3b frameL);
        void drawTools_smooth(cv::Mat3b frameL, vector<cv::Mat> &toolTvec_smooth, vector<cv::Mat> &toolRvec_smooth);

        void toolPose2cvTrans(Marker toolp, cv::Mat &trans);
        void cvTrans2toolPose(cv::Mat trans, Marker &toolp);

        int CameraL_ind, CameraR_ind;
        string theIntrinsicFileL, theIntrinsicFileR;

        int ThePyrDownLevel;
        myaruco::MarkerDetector MDetector;
        myaruco::MarkerDetector MDetectorL, MDetectorR;

        std::vector<aruco::Marker> TheMarkers;
        std::vector<aruco::Marker> TheMarkersL, TheMarkersR;
        std::vector<aruco::Marker> StereoMarkersL, StereoMarkersR, StereoMarkersL_pre;

//        vector<int> ToolID;
//        aruco::Marker Tool;
        vector<aruco::Marker> Tools;
        vector<vector<int> > ToolsID;
        float TheMarkerSize;
        vector<float> MarkersSize;
//        int MarkerSize;
        cv::Mat Q, mx1, my1, mx2, my2, P1, P2, T, R, D1, D2;
        std::vector<cv::Point3f> markercornerLocal;

        cv::Mat TheInputImage,TheInputImageCopy;
        aruco::CameraParameters TheCameraParameters;
        aruco::CameraParameters TheCameraParametersL, TheCameraParametersR;
        aruco::CameraParameters StereoCameraParametersL, StereoCameraParametersR;
        vector<bool> toolsDetected;
        int filterwindow;

    private:
        int numMarkers;
        int numTools;


    };


}
