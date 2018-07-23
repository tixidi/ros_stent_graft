#define _USE_MATH_DEFINES

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <highgui.h>

class needlePoseEstimation {
		
	public:

        cv::Mat P1, P2, D1, D2;
        cv::Mat cHl, lHn, cHr, rHn;
        void FindThread( IplImage * leftImage_in, IplImage * rightImage_in, int* X_l, int * X_r);
        void estimatePose(IplImage * leftImage_in, IplImage * rightImage_in, std::vector<cv::Point3d> &needle3DModel_inout, cv::Mat * needlePose_inout , int toolIndx);
        void printNeedlePntsGlobalImage(cv::Mat frameL, cv::Mat frameR, std::vector<cv::Point3d> needlepointsToolL, int toolIndx, int r, int g, int b);
        IplImage *energyIPL_L, *energyIPL_R;
        cv::Mat energyMat_L, energyMat_R;
        std::vector<double> energy_L, energy_R;
        void printNeedleSearchRange(cv::Mat frameL, cv::Mat frameR, std::vector<cv::Point3d> needlepointsOpt,
                                    int toolIndx, int r, int g, int b);
        void HessianAnalysisEigenvalues( IplImage * imageGray_in, double sigma_in, int kernelDimension_in, IplImage * imageGray_out );
        std::vector<std::vector<cv::Point3d> > needle3D_all;

	private:
        IplImage * leftImageFeature;
        IplImage * rightImageFeature;
		IplImage * leftImageFeatureDistance;
		IplImage * rightImageFeatureDistance;
        IplImage * leftImage;
        IplImage * rightImage;

		void calcFeatureImages( IplImage * leftImage_in, IplImage * rightImage_in, IplImage * leftImageDistance_in, IplImage * rightImageDistance_in );
        void calcEnergy(std::vector<cv::Point3d> needle3DModelCurrentPose_in, double * energy_out , int toolIndx);
        void calcEnergy2(std::vector<cv::Point3d> needle3DModelCurrentPose_in,
                          int toolIndx );
        void optimisation(std::vector<cv::Point3d> &needle3DModel_inout, cv::Mat * toolPose_inout , int toolIndx);

        void projectNeedlePntsFrameLR(std::vector<cv::Point3d> needlepointsToolL,
                                      std::vector<cv::Point3d> &needle2camera,
                                      std::vector<cv::Point2d> &needle2imgL,
                                      std::vector<cv::Point2d> &needle2imgR, int toolIndx);




		void calculateKernel( double sigma_in, int kernelDimension_in, int typeKernel_in,  CvMat * Kernel_out );

};

