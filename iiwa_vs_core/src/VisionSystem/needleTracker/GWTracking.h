#ifndef GWTRACKING_H
#define GWTRACKING_H

#include "mainFunctions.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv/cv.h"
#include <opencv2/imgproc/imgproc.hpp>

class GWTracking{
public:
	
	GWTracking();
	GWTracking( std::string seqName_in, int indexFirstFrame_in, int startingPointInstrument_in, int verbose_in, double sigmaHessian_in );
	
	void initialization( std::vector<Point2d> GWInit_in, IplImage * currFrame3c_in, IplImage * currFrame1c_in );
	void runTracking( int indexFrame_in, IplImage * currFrame3c_in, IplImage * currFrame1c_in, std::vector<Point2d> &GWInit_out );
	
private:
	
	int indexFirstFrame;
	int startingPointInstrument;
	int indexFrame;
	int verbose; 

	std::string seqName;
	Rect roi;

	//Hessian - Lineness 
	int kernelDimension;
	double sigmaHessian;
	float minLinenessValue;

	//Image Processing
	int minSizeSegment; int maxNumPixelPerSegment;	double slopeThreshold;
	int numPixelTip;

	//SEGlets
	int lengthSegmentForCandidates;				
	float maxDisplacementSegments; float maxChangeInOrientationSegments; float maxChangeInLengthSegments;
	float weightBG; float weightDistance; float weightOrientation; float weightLength;

	//Generate Tracking Hypothese
	float maxDistance_Base; float maxOrientation_Base; 
	float distanceThreshold_Tip; float minDistanceThreshold_Tip; float angleThreshold_Tip;
	float distanceThreshold_Seg; float minDistanceThreshold_Seg; float angleThreshold_Seg;

	//Background
	float gain;

	//Evaluate Tracking Hypotheses
	int distanceNodes;
	int lengthSegmentForIntensityMeasure_GW; int lengthSegmentForIntensityMeasure_Normal; int distanceNormalPatterns;	
	double memoryModel;
	double sigmaPrior;
	double meanLinenessModel;
	double sigmaLinenessModel;
	double weightMeasures[2];
	
	IplImage *currFrame3c;
	IplImage *prevFrame3c;
	IplImage *prevFrame1c;
	IplImage *currFrame1c;
	IplImage *currFrameMaskTool_1;
	IplImage *currFrameMaskTool_2;
	IplImage *prevLineness;
	IplImage *currLineness;
	IplImage *backgroundMap;
	
	std::vector<Point2d> currGW;
	std::vector<Point2d> prevGW;
	std::vector<Point2d> sparseGrid;

	std::vector<Point2d> curvePatternCurrentHp;
	std::vector<Point2d> normalPattern1CurrentHp;
	std::vector<Point2d> normalPattern2CurrentHp;

    std::vector<vector<int> > hps;
	std::vector<int> indexTipSegment;
	std::vector<straightSegment> possibleSegmentsBase;

	void imageProcessing( std::vector<straightSegment> &listSegmentsGT_out, std::vector<straightSegment> &listSegmentsFilteredOut_out, std::vector<straightSegment> &listSegments_out );
	void detectSEGlets( std::vector<straightSegment> listSegmentsGT_in, std::vector<straightSegment> listSegments_in, std::vector<straightSegment> listSegmentsFilteredOutTooFar_in, std::vector<straightSegment> &SEGlets_out, std::vector<straightSegment> &listSegmentsFilteredOut_out );
	void generateTrackingHps( std::vector<Point2d> centrelineGT_in, std::vector<straightSegment> SEGlets_in, std::vector<trackingHp> &trackingHps_out );
	void evaluateTrackingHps( IplImage *prevImage_in, IplImage *currentImage_in, IplImage *prevImageColor_in, IplImage *currentImageColor_in, IplImage *prevImageLineness_in, IplImage *currentImageLineness_in, std::vector<Point2d> centrelineGT_in, std::vector<trackingHp> trackingHps_in );

	void reinitializeVariables();

    void toolsDetection();
	
	void linenessCurrentFrame();
	void binarizeCurrentLineness( IplImage *currBinaryFrame_out );
	void extractStraightSegmentsGT( std::vector<straightSegment> &listSegmentsGT_out );
	void extractStraightSegments( IplImage *currBinaryFrame_in, std::vector<straightSegment> &listSegmentsFilteredOut_out, std::vector<straightSegment> &listSegments_out );
	
    void chooseSEGlets( std::vector<vector<matchingCost> > matchingCosts_in, std::vector<straightSegment> listSegmentsGT_in, std::vector<straightSegment> listSegments_in, std::vector<straightSegment> &SEGlets_out, std::vector<straightSegment> &listSegmentsFilteredOut_out );
    void calculateMatchingCosts( std::vector<straightSegment> listSegmentsGT_in, std::vector<straightSegment> listSegments_in, std::vector<vector<matchingCost> > &matchingCosts_out);
	void calculateSingleCost( std::vector<Point2d> prevSegment_in, std::vector<Point2d> nextSegment_in, matchingCost *cost_out);
	
    void calculatePossiblePaths( std::vector<straightSegment> SEGlets_in, std::vector<Point2d> &firstSegment_out, std::vector<vector<Point2d> > &pointsHps_out );
	void addBaseGW( std::vector<Point2d> firstSegment_in, bool *foundAtLeast1_out, straightSegment *bestSegmentToAdd_out );
    void createCurvesFromHps( bool foundAtLeast1_in, std::vector<vector<Point2d> > pointsHps_in, straightSegment bestSegmentToAdd_in, std::vector<trackingHp> &trackingHps_out );
    int findSegmentConnections( std::vector<vector<Point2d> > availableSegments_in, std::vector<Point2d> currentSegment_in, int currentIndexNode_in, std::vector<int> currentHypothesis_in );
	int areConnectedTipSegments(std::vector<Point2d> currentSegment_in, std::vector<Point2d> possibleMatch_in, double *distanceSegments_out );
	int areConnected(std::vector<Point2d> currentSegment_in, std::vector<Point2d> possibleMatch_in, double *distanceSegments_out );
    void filterConnectedSegments( std::vector<int> &indexConnectedSegments_inout, std::vector<vector<Point2d> > &connectedSegments_inout, std::vector<double> &segmentDistances_inout );
	void calculateBackgroundMap( int indexFrame_in, std::vector<Point2d> centrelineGT_in, std::vector<straightSegment> listSegments_in );

	void calculateGWModel( std::vector<Point2d> centrelineGT_in, IplImage *prevImage_in, IplImage *prevImageColor_in, trackingHp *model_out );
	void calculateLocalBinaryPatterns( int indexFrame_in, trackingHp &currentHp_in, IplImage *currentImage_in );
	void calculateNormalPattern( int indexFrame_in, Point2d startingPoint_in, std::vector<Point2d> curve_in, IplImage *currentImage_in, double *normalPattern1_out, double *normalPattern2_out );
	void updateLinenessModel( int indexHp_in, trackingHp bestHP_in, trackingHp model_in, IplImage *currentImageLineness_in, IplImage *prevImageLineness_in );
	void evaluateLikelihoodCurrentHp( ofstream &myfile_in, int indexHp_in, trackingHp currentHp_in, trackingHp model_in, IplImage *prevImage_in, IplImage *currentImage_in, IplImage *prevImageLineness_in, IplImage *currentImageLineness_in, double *energyHp_out );
	void calculatePriorCurrentHp( trackingHp currentHp_in, trackingHp model_in, double *prior_out );
	void calculateIntensityPatternCurrentHp( trackingHp currentHp_in, trackingHp model_in, double *internsityPatternMeasure_out );
	void calculateLinenessMeasureCurrentHp( int indexHp_in, trackingHp currentHp_in, IplImage *currentImageLineness_in, double *linenessMeasure_out );

	void loadCurrentFrame();
};

#endif
