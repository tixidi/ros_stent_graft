#ifndef NEEDLEDRIVERTRACKING_H
#define NEEDLEDRIVERTRACKING_H
#include<QThread>
#include"patterntracker/tracker.h"
#include"patterntracker/tracker_keydot.h"
#include"definitionsVision.h"
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include "handEyeAbsoluteOrientation/AbsoluteOrientation.h"
//#include "tracktools/tracktools.h"

class NeedleDriverTracking : public QThread
{
   Q_OBJECT
private:
    Mat Q, mx1, my1, mx2, my2, P1, P2;
    dvrk::TrackerKeydot *trackerLeft;
    dvrk::TrackerKeydot *trackerRight;
    int nx ;    int ny ;
    float keydot_square_size ;
public:
    NeedleDriverTracking();
    bool findPatternPose( vector<cv::Point2f> &pointsLeft, vector<cv::Point2f> &pointsRight, cv::Mat & poseCameraFrame);
    //bool findPatternPose(  stereoImages & Images,  cv::Mat & poseCameraFrame);
    bool KeyDotsObserved( stereoImages & Images,vector<cv::Point2f> &cornersLeft, vector<cv::Point2f> &cornersRight);
    //stereoImages drawKeyDotsOnStereo(stereoImages &Images);
    stereoImages drawKeyDotsOnStereo(stereoImages &Images, vector<cv::Point2f> &pointsLeft, vector<cv::Point2f> &pointsRight);
};
#endif
