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

#include <opencv2/video/tracking.hpp>
#include "../patterntracker/tracker_general.h"
#include "../patterntracker/tracker.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/v4l2-mediabus.h>

namespace myaruco{

    class TrackTools
    {
    public:
        TrackTools(string fileDir);
//        void initTool(float markerSize, vector<int> toolid);
        void initTools(vector<float> markersSizes, vector<vector<int> > toolsids, int fWindow);
        void initTrackers();
        bool findStereoMarkers();
        void findStereoMarkers_tracker(vector<bool> markersL_bool, vector<bool> markersR_bool);
        void filterStereoMarkers(vector<bool> markersL_bool, vector<bool> markersR_bool);
        void estimateStereoMarkerPose();
        bool estimateToolPose(aruco::Marker &tool, vector<int> toolID, int N, float size);
        bool estimateToolPose_multimarkers(aruco::Marker &tool, vector<int> toolID, int N, float markersize);
        float markerAera(int index);
        void detectTools(cv::Mat3b frameL, cv::Mat3b frameR,
                         vector<cv::Mat> &toolsTvec,
                         vector<cv::Mat> &toolsRvec,
                         vector<bool> &detected);
        void trackTools(cv::Mat3b frameL, cv::Mat3b frameR,
                        vector<cv::Mat> &toolsTvec,
                        vector<cv::Mat> &toolsRvec,
                        vector<bool> &detected);
        vector<bool> trackMarkers(cv::Mat &track_image, vector<Marker> &detectedMarkers,
                                    vector<Marker> &toolMarkers, vector<dvrk::TrackerGeneral> &trackers,
                                    CameraParameters camParams, float markerSizeMeters);
        void filterStereoMarkers(vector<bool> markers_bool,
                                 vector<Marker> &Markers_track,
                                 vector<vector<Marker> > &Markers_hist,
                                 vector<Marker> &Markers_detect, CameraParameters camParams, float markerSizeMeters);

        void drawStereoMarkers(cv::Mat3b frameL, cv::Mat3b frameR);
        void drawTools(cv::Mat3b frameL);
        void drawTools_smooth(cv::Mat3b frameL, vector<cv::Mat> &toolTvec_smooth, vector<cv::Mat> &toolRvec_smooth);
        void drawNeedle(cv::Mat3b frameL, bool bool_frameL, cv::Mat ToolLHNeedle, cv::Mat ToolLHNeedleTip, int toolIndex);
        void drawTip(cv::Mat3b frameL, bool bool_frameL, cv::Mat ToolLHNeedle, cv::Mat ToolLHNeedleTip);

        void computeNeedlePntsToolL(vector<cv::Point3d> needlepoints,
                                    cv::Mat lHn,
                                    vector<cv::Point3d> &needlepointsToolL);

        vector<cv::Point3d> NeedlePnts;

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

        //For tracking
        vector<Marker>  MarkersL_track, MarkersR_track;
        vector<dvrk::TrackerGeneral> trackersL, trackersR;
        vector<vector<Marker> > MarkersL_hist, MarkersR_hist;

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

        vector<aruco::Marker> Tools_kalman;
        void drawKalmans(cv::Mat3b frameL);

    private:
        int numMarkers;
        int numTools;
        int numMarkers_tool;
        int filterwindow;
        int fcount;
        vector<vector<aruco::Marker> > Tools_hist;

        vector<aruco::Marker>  Tools_previous;
        vector<bool> fristDetected;
        vector<cv::KalmanFilter> KFtool, KFtool_rot;

        void initializeKF(cv::KalmanFilter &kf, cv::KalmanFilter &kf_rot);
        void kalmanFilterPose(Marker &pose_detected, Marker &pose_previous, Marker &pose_filtered,
                              cv::KalmanFilter &KF, cv::KalmanFilter &KF_rot, int tool_ind);

    };


}
