#ifndef NEEDLEPOSEESTIMATION_H
#define NEEDLEPOSEESTIMATION_H

#include "globals.h"
#include"needleTracker/mainFunctions.h"
#include"needleTracker/GWTracking.h"
#include"KUKAControl/definitions.h"
class NeedlePoseEstimation
{
public:
    NeedlePoseEstimation();
    bool doPoseEstimation(cv::Mat leftImage, cv::Mat rightImage, cv::Mat &estNeedlePose, std::vector<cv::Point3d> &needle3Dmodel, cv::Mat &needleDriverPose, cv::Mat &finalNeedlePose);
    bool poseEstimation3D( std::vector<cv::Point3d> &needle3Dmodel, cv::Mat &needleDriverPose, std::vector<cv::Point3d> &needlePoints, cv::Mat &finalNeedlePose);

private:
    GWTracking needleDectionLeft;
    GWTracking needleDectionRight;

    bool pointMatch_rigid2(std::vector<Point3d> &m_R, std::vector<Point3d> &m_C, cv::Mat &R_C);
    bool needleReconstruction3D(cv::Mat leftImage, cv::Mat rightImage, std::vector<cv::Point2d> &needlePointsLeft, std::vector<cv::Point2d> &needlePointsRight, std::vector<cv::Point3d> &needlePoints, int width_image);
    bool project3DModel( cv::Mat leftImage, cv::Mat rightImage, std::vector<cv::Point3d> &needle3Dmodel, std::vector<cv::Point2d> &needlePointsLeft_estimate, std::vector<cv::Point2d> &needlePointsRight_estimate);
    bool project3DModel2( cv::Mat leftImage, cv::Mat rightImage, std::vector<cv::Point3d> &needle3Dmodel, cv::Mat newNeedlePose,std::vector<cv::Point3d> &needlePoints);
};

template <typename CV_POINT3_SRC, typename CV_POINT3_DST>
static void find_rigidTransformation2(
const std::vector<CV_POINT3_SRC> &_src,
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

#endif
