/*
    TrackerPattern class derived from Tracker

    Dectecting pattern

    2015 Lin Zhang, Menglong Ye
    The Hamlyn Centre for Robotic Surgery,
    Imperial College, London
*/

#ifndef TRACKER_PATTERN_H
#define TRACKER_PATTERN_H

#include <opencv2/opencv.hpp>
#include "tracker.h"

namespace dvrk {

class Tracker_Pattern : public Tracker
{
public:
    typedef enum{
        DOT_PATTERN,
        CHESS_PATTERN
    } PatternType;

    Tracker_Pattern();
    Tracker_Pattern(const cv::Size &pat_size, PatternType pat_type);
    virtual ~Tracker_Pattern() {}

    void initTrack(const cv::Mat &I);

    bool track(const cv::Mat &I);

    inline cv::Size2i &getPatternSize() {return pattern_size;}

    // get points in image coordinate
    std::vector<cv::Point2f> getP_img();
    // get points in normalized meter coordinate provided camera parameters
    std::vector<cv::Point2d> getP_norm(
        const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs);

private:
    // Detected pattern corners/centres (pixel coordinate)
    std::vector<cv::Point2f> P_img;

    // Detected pattern in camera coordinate (normalized meters)

    // Pattern info
    cv::Size pattern_size;
    PatternType pattern_type;

    // Blob detection for dot pattern
    cv::Ptr<cv::FeatureDetector> blobDetector;

};

}   // namespace dvrk

#endif  // TRACKER_PATTERN_H
