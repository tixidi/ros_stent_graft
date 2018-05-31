#ifndef MAINFUNCTIONS_H
#define MAINFUNCTIONS_H

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <cvblob.h>

#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stack>

/*#include <gsl/gsl_bspline.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_statistics.h>*/

using namespace cv;
using namespace std;
using namespace cvb;

struct straightSegment  
{
	int id;
	std::vector<Point2d> points;	
};

struct matchingCost 
	{
		int index;
		float cost; 
		matchingCost() : index(-1), cost(0.0) { }
		matchingCost(int i, float c) : index(i), cost(c) { } 
		bool operator() (const matchingCost &cost1, const matchingCost &cost2) {
		return (cost1.cost < cost2.cost);
		}
};

struct trackingHp  
{
	int id;
	std::vector<int> indexes;
	std::vector<Point2d> points;	
	
	std::vector<double> curvePattern;
	std::vector<double> normalPattern1;
	std::vector<double> normalPattern2;

	std::vector<double> Spline_Pattern;
	std::vector<double> Bar_Pattern;
	std::vector<double> SLPB_Pattern;
};

void calculateKernel( float sigma_in, int kernelDimension_in, int typeKernel_in,  CvMat * Kernel_out);
void HessianAnalysisEigenvaluesStandard ( IplImage * imageGray_in, float sigma_in, int kernelDimension_in, IplImage * imageGray_out );

void thinningIteration(cv::Mat& img, int iter);
void thinning(const cv::Mat& src, cv::Mat& dst);

void calculateDistanceSegments(std::vector<Point2d> prevSegment_in, std::vector<Point2d> nextSegment_in, float *distance_out);
void findOrientationCloudPoints( std::vector<Point2d> points_in, Point2d *directions_out, float *orientation_out );
void findOrientationCloudPointsSorted( std::vector<Point2d> points_in, Point2d *directions_out, float *orientation_out );

void refineSegmentsTip ( int minSizeSegment_in, int numPixelTip_in, std::vector<straightSegment> &listSegments_inout );
void extractStraightSegmentsFromAConnectedRegion( int maxNumPixelPerSegment_in, double slopeThreshold_in, IplImage *image_in, std::vector<Point2d> blob_in, int *currentIndexSegment_in, int minSizeSegment_in, std::vector<straightSegment> &currentListSegments_out );
void extractStraightSegmentsFromAConnectedRegionGT( int maxNumPixelPerSegment_in, double slopeThreshold_in, std::vector<Point2d> blob_in, int *currentIndexSegment_in, int minSizeSegment_in, std::vector<straightSegment> &currentListSegments_out );
void regionGrowing ( int maxNumPixelPerSegment_in, double slopeThreshold_in, IplImage * image_in, IplImage * exploringMask_in, std::vector<Point3d> seeds_in, std::vector<Point3d> &resultRegionGrowing_out, std::vector<Point3d> &lastNeighborhood_out );
void extractBlobs( int minSizeSegment_in, int minDistCentrelineGT_in, IplImage *image_in, std::vector<Point2d> centrelineGT_in, std::vector<vector<Point2d> > &listBlobs_out, std::vector<straightSegment> &listSegmentsFilteredOut_out );
void fromBlobLabelsToPoints(IplImage *labelImg_in, CvBlobs blobs_in, std::vector<vector<Point2d> > &listBlobs_in);

void bsplineCubic2D(int numPoints_in, int numCoeffs_in, int numPoints_out, std::vector<double> pointToFitX_in, std::vector<double> pointToFitY_in, std::vector<double> &pointsFittedX_out, std::vector<double> &pointsFittedY_out);
//void createSmoothCurveFromPoints( int distanceNodes_in, std::vector<Point2d> &pointsHp_inout, int indexCurve_in, IplImage *currentImage_in, int startingPointInstrument_in, std::vector<int> &indexCurve_out );
void createSmoothCurveFromPoints2( int distanceNodes_in, std::vector<Point2d> &pointsHp_inout, int indexCurve_in, IplImage *currentImage_in, int startingPointInstrument_in, std::vector<int> &indexCurve_out );

void read2DCenterlineFromFile( std::vector<Point2d> &centerline_out, int *numPoint_in, std::string Filename_in, int startingPointInstrument_in );
void printLinenessMap( IplImage *currentImage_in, std::string fileName_in, int indexFrame_in );
void printBinaryImage( IplImage *currentImage_in, std::string fileName_in, int indexFrame_in );
void printSegments( IplImage *currentImage_in, int type_in, std::string fileName_in, int indexFrame_in, std::vector<straightSegment> listSegments_in );
void printBackgroundMap( IplImage *currentImage_in, std::string fileName_in, int indexFrame_in );
void printSegmentsHps( IplImage *currentImage_in, std::string fileName_in, int indexFrame_in, std::vector<straightSegment> listSegments_in, std::vector<vector<int> > hps_in );
void printCurveHps( IplImage *currentImage_in, std::string fileName_in, int indexFrame_in, std::vector<trackingHp> trackingHps );
void printLocalBinaryPatterns( IplImage *currentImage_in, int type_in, std::string fileName_in, int indexFrame_in, trackingHp currentHp_in, std::vector<Point2d> curvePatternCurrentHp_in, std::vector<Point2d> normalPattern1CurrentHp_in, std::vector<Point2d> normalPattern2CurrentHp_in );
void printTrackingResult(  IplImage *currentImage_in, std::string fileName_in, int indexFrame_in, std::vector<Point2d> GW_in, int indexBestTrackingHypothesis_in );
void printGWModel( IplImage *currentImage_in, std::string fileName_in, int indexFrame_in, trackingHp model_in );
void printFirstFrame( IplImage *currentImage_in, std::string fileName_in, std::vector<Point2d> GW_in );

CvScalar random_color(CvRNG* rng);

#endif
