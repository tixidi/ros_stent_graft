#include "GWTracking.h"

GWTracking::GWTracking()
{
}

GWTracking::GWTracking( std::string seqName_in, int indexFirstFrame_in, int startingPointInstrument_in, int verbose_in, double sigmaHessian_in )
{
	seqName	= seqName_in;
	indexFirstFrame = indexFirstFrame_in;
	indexFrame = indexFirstFrame_in;
	startingPointInstrument = startingPointInstrument_in;
	verbose = verbose_in;	

	currFrame3c = NULL;	prevFrame3c = NULL;	prevFrame1c = NULL;	currFrame1c = NULL;	prevLineness = NULL; currLineness = NULL; backgroundMap = NULL;

	//Hessian - Lineness 
	sigmaHessian = sigmaHessian_in;
	kernelDimension = 13;
    minLinenessValue = 2.0;

	//Image Processing
    //minSizeSegment = 10; maxNumPixelPerSegment = 20; slopeThreshold = 50.0;
    //minSizeSegment = 4; maxNumPixelPerSegment = 20; slopeThreshold = 50.0;
    minSizeSegment = 2; maxNumPixelPerSegment = 20; slopeThreshold = 50.0;
	numPixelTip = 20;
	
	//SEGlets
	lengthSegmentForCandidates = 30;				
    //maxDisplacementSegments = 30; maxChangeInOrientationSegments = 40;	maxChangeInLengthSegments = 0.9;
    //maxDisplacementSegments = 50; maxChangeInOrientationSegments = 40;	maxChangeInLengthSegments = 0.9;
    maxDisplacementSegments = 25; maxChangeInOrientationSegments = 40;	maxChangeInLengthSegments = 0.9;
	weightBG = 0.0; weightDistance = 0.333; weightOrientation = 0.333; weightLength = 0.333;

	//Generate Tracking Hypothese
    //maxDistance_Base = 20; maxOrientation_Base = 10;
    maxDistance_Base = 20; maxOrientation_Base = 50;
	distanceThreshold_Tip = 40; minDistanceThreshold_Tip = 3; angleThreshold_Tip = 25;
	distanceThreshold_Seg = 70; minDistanceThreshold_Seg = 3; angleThreshold_Seg = 60;

	//Background
	gain = 1;

	//Evaluate Tracking Hypotheses
	distanceNodes = 5;
	lengthSegmentForIntensityMeasure_GW = 5; lengthSegmentForIntensityMeasure_Normal = 5; distanceNormalPatterns = 4;	
	memoryModel = 0.8;
	sigmaPrior = 25;
	weightMeasures[0] = 0.5; weightMeasures[1] = 0.5;					
}

void GWTracking::initialization( std::vector<Point2d> GWInit_in, IplImage * currFrame3c_in, IplImage * currFrame1c_in )
{
	int offset = 20;
	roi.x = offset;		
	roi.y = offset;
	roi.width = currFrame3c_in->width - offset;	
	roi.height = currFrame3c_in->height - offset;

	//Copy GW points from manual initialization
	currGW = GWInit_in;

	//Load first frame
	currFrame3c = currFrame3c_in;
	currFrame1c = currFrame1c_in;

	//Allocate memory for lineness images
	prevLineness = cvCreateImage( cvGetSize(currFrame1c), IPL_DEPTH_32F, 1);
	currLineness = cvCreateImage( cvGetSize(currFrame1c), IPL_DEPTH_32F, 1);

	//Calculate lineness image for first frame
	HessianAnalysisEigenvaluesStandard ( currFrame1c, sigmaHessian, kernelDimension, currLineness );
	
	//Initialize Background Map
	backgroundMap = cvCreateImage( cvGetSize(currFrame1c), IPL_DEPTH_32F, 1);
	cvSet(backgroundMap, cvScalar(0));

    currFrameMaskTool_1 = cvCreateImage( cvGetSize(currFrame1c), IPL_DEPTH_8U, 1);
    currFrameMaskTool_2 = cvCreateImage( cvGetSize(currFrame1c), IPL_DEPTH_8U, 1);

	if( verbose != 0 )
		printFirstFrame( currFrame3c, seqName, currGW );
}

void GWTracking::runTracking( int indexFrame_in, IplImage * currFrame3c_in, IplImage * currFrame1c_in, std::vector<Point2d> &GWInit_out )
{
	std::vector<straightSegment> listSegmentsFilteredOutTooFar;
	std::vector<straightSegment> listSegmentsFilteredOut;
	std::vector<straightSegment> listSegmentsGT;
	std::vector<straightSegment> listSegments;
	std::vector<straightSegment> SEGlets;	
	std::vector<trackingHp> trackingHps;

	indexFrame = indexFrame_in;
	
	reinitializeVariables();
	currFrame3c = currFrame3c_in;
	currFrame1c = currFrame1c_in;

    toolsDetection();

	imageProcessing( listSegmentsGT, listSegmentsFilteredOutTooFar, listSegments );
	detectSEGlets( listSegmentsGT, listSegments, listSegmentsFilteredOutTooFar, SEGlets, listSegmentsFilteredOut);
	generateTrackingHps( prevGW, SEGlets, trackingHps );
	evaluateTrackingHps( prevFrame1c, currFrame1c, prevFrame3c, currFrame3c, prevLineness, currLineness, prevGW, trackingHps );

	GWInit_out = currGW;
}

void GWTracking::reinitializeVariables()
{
	curvePatternCurrentHp.clear();
	normalPattern1CurrentHp.clear();
	normalPattern2CurrentHp.clear();
	indexTipSegment.clear();
	hps.clear();
	possibleSegmentsBase.clear();

	//if(prevFrame3c != NULL)	cvReleaseImage(&prevFrame3c);
	//if(prevFrame1c != NULL)	cvReleaseImage(&prevFrame1c);

	prevFrame3c = currFrame3c;
	prevFrame1c = currFrame1c;
	prevLineness = currLineness;
	
	prevGW = currGW;
	currGW.clear();
}

void GWTracking::imageProcessing( std::vector<straightSegment> &listSegmentsGT_out, std::vector<straightSegment> &listSegmentsFilteredOutTooFar_out, std::vector<straightSegment> &listSegments_out )
{
	IplImage *currBinaryFrame = cvCreateImage( cvGetSize(currFrame1c), IPL_DEPTH_8U, 1);
	cvSet(currBinaryFrame, cvScalar(0));

	linenessCurrentFrame();
	binarizeCurrentLineness( currBinaryFrame );
	extractStraightSegmentsGT( listSegmentsGT_out );
	extractStraightSegments( currBinaryFrame, listSegmentsFilteredOutTooFar_out, listSegments_out );

	if( verbose != 0 )
	{
		printLinenessMap( currLineness, seqName, indexFrame );
		printBinaryImage( currBinaryFrame, seqName, indexFrame );

		//if type = 0 => Segments = GT;
		printSegments( currFrame3c, 0, seqName, indexFrame, listSegmentsGT_out  );
		//if type = 1 => Segments = listSegments;
		printSegments( currFrame3c, 1, seqName, indexFrame, listSegments_out  );
		//if type = 2 => Segments = listSegmentsFilteredOutTooFar;
		printSegments( currFrame3c, 2, seqName, indexFrame, listSegmentsFilteredOutTooFar_out  );
	}

	cvReleaseImage(&currBinaryFrame);
}

void GWTracking::detectSEGlets( std::vector<straightSegment> listSegmentsGT_in, std::vector<straightSegment> listSegments_in, std::vector<straightSegment> listSegmentsFilteredOutTooFar_in, std::vector<straightSegment> &SEGlets_out, std::vector<straightSegment> &listSegmentsFilteredOut_out )
{
    std::vector<vector<matchingCost> > matchingCosts;
	std::vector<straightSegment> backgroundSegments;

	backgroundSegments = listSegmentsFilteredOutTooFar_in;
	
	calculateMatchingCosts( listSegmentsGT_in, listSegments_in, matchingCosts);
	chooseSEGlets( matchingCosts, listSegmentsGT_in, listSegments_in, SEGlets_out, listSegmentsFilteredOut_out );

	//Collect all the segments that are not SEGlets to update the background map
	for(int i=0; i<listSegmentsFilteredOut_out.size(); i++) 
		backgroundSegments.push_back(listSegmentsFilteredOut_out[i]);

	calculateBackgroundMap( indexFrame, prevGW, backgroundSegments );

	if( verbose != 0 )
	{
		//if type = 3 => Segments = listSegments;
		printSegments( currFrame3c, 3, seqName, indexFrame, listSegmentsFilteredOut_out );
		//if type = 4 => Segments = SEGlets;
		printSegments( currFrame3c, 4, seqName, indexFrame, SEGlets_out  );

		printBackgroundMap( backgroundMap, seqName, indexFrame );
	}
}

void GWTracking::generateTrackingHps( std::vector<Point2d> centrelineGT_in, std::vector<straightSegment> SEGlets_in, std::vector<trackingHp> &trackingHps_out )
{	
    std::vector<vector<Point2d> > pointsHps;
	std::vector<Point2d> firstSegment;
	straightSegment bestSegmentToAddBase;
	bool foundAtLeast1 = false;

	calculatePossiblePaths( SEGlets_in, firstSegment, pointsHps );
    addBaseGW( firstSegment, &foundAtLeast1, &bestSegmentToAddBase );
	createCurvesFromHps( foundAtLeast1, pointsHps, bestSegmentToAddBase, trackingHps_out );

	if( verbose != 0 )
	{
		printSegmentsHps( currFrame3c, seqName, indexFrame, SEGlets_in, hps );
		printCurveHps( currFrame1c, seqName, indexFrame, trackingHps_out );
	}
}

void GWTracking::evaluateTrackingHps( IplImage *prevImage_in, IplImage *currentImage_in, IplImage *prevImageColor_in, IplImage *currentImageColor_in, IplImage *prevImageLineness_in, IplImage *currentImageLineness_in, std::vector<Point2d> centrelineGT_in, std::vector<trackingHp> trackingHps_in )
{
	trackingHp model;
	std::vector<double> energyHps;
	std::stringstream ss;
	std::string imageName;	
	ofstream myfile;

	ss << seqName << "HpsCurves/HpsScores_Frame" << indexFrame << ".txt";
	imageName = ss.str();
	const char * imageNameChar = imageName.c_str();
//	if(verbose != 0)
//		myfile.open(imageName);

	//Calculate the GW model based on the previous GW tracking result
	calculateGWModel( centrelineGT_in, prevImage_in, prevImageColor_in, &model );
	
	//Create the normal distribution that describes the lineness of the GW
	if( indexFrame-1 == indexFirstFrame )
		updateLinenessModel( 0, trackingHps_in[0], model, currentImageLineness_in, prevImageLineness_in );

	//Evaluate the likelihood of each hypotheses
    int indexBestHp = 0;
	double maxLikelihood = DBL_MIN;
	for (int i=0; i < trackingHps_in.size(); i++ )
	{
		double likelihoodCurrentHp;

		curvePatternCurrentHp.clear(); normalPattern1CurrentHp.clear();	normalPattern2CurrentHp.clear();
		calculateLocalBinaryPatterns( indexFrame, trackingHps_in[i], currentImage_in );		

		if( verbose != 0 )
			printLocalBinaryPatterns( currentImageColor_in, 1, seqName, indexFrame, trackingHps_in[i], curvePatternCurrentHp, normalPattern1CurrentHp, normalPattern2CurrentHp );

		evaluateLikelihoodCurrentHp( myfile, i, trackingHps_in[i], model, prevImage_in, currentImage_in, prevImageLineness_in, currentImageLineness_in, &likelihoodCurrentHp );
		
		energyHps.push_back(likelihoodCurrentHp);

		if( maxLikelihood < likelihoodCurrentHp)
		{
			maxLikelihood = likelihoodCurrentHp;
			indexBestHp = i;
		}
	}

	if(verbose != 0)
		myfile.close();

	//Save the tracking result that has the highest likelihood
	currGW = trackingHps_in[indexBestHp].points;

	updateLinenessModel( 2, trackingHps_in[indexBestHp], model, currentImageLineness_in, prevImageLineness_in );

	if( verbose != 0 )
	{
		printGWModel( prevImage_in, seqName, indexFrame, model );
		printTrackingResult(  currFrame3c, seqName, indexFrame, currGW, indexBestHp );
	}

	printTrackingResult(  currFrame3c, seqName, indexFrame, currGW, indexBestHp );
}

void GWTracking::loadCurrentFrame()
{
	std::stringstream ss, ss1, ss2;
	std::string imageName, imageName1, imageName2;

	if(indexFrame<10)
		ss << seqName << "Frames/frame000" << indexFrame << ".png";
	else if(indexFrame<100)
		ss << seqName << "Frames/frame00" << indexFrame << ".png";
	else if(indexFrame<1000)
		ss << seqName << "Frames/frame0" << indexFrame << ".png";
	else
		ss << seqName << "Frames/frame" << indexFrame << ".png";

	if(indexFrame<10)
		ss1 << seqName << "Frames_tool1/frame000" << indexFrame << ".png";
	else if(indexFrame<100)
		ss1 << seqName << "Frames_tool1/frame00" << indexFrame << ".png";
	else if(indexFrame<1000)
		ss1 << seqName << "Frames_tool1/frame0" << indexFrame << ".png";
	else
		ss1 << seqName << "Frames_tool1/frame" << indexFrame << ".png";

	if(indexFrame<10)
		ss2 << seqName << "Frames_tool2/frame000" << indexFrame << ".png";
	else if(indexFrame<100)
		ss2 << seqName << "Frames_tool2/frame00" << indexFrame << ".png";
	else if(indexFrame<1000)
		ss2 << seqName << "Frames_tool2/frame0" << indexFrame << ".png";
	else
		ss2 << seqName << "Frames_tool2/frame" << indexFrame << ".png";

	imageName = ss.str();
	imageName1 = ss1.str();
	imageName2 = ss2.str();
	
	const char * imageNameChar = imageName.c_str();
	const char * imageNameChar1 = imageName1.c_str();
	const char * imageNameChar2 = imageName2.c_str();

    currFrame3c = cvLoadImage(imageNameChar);
    currFrame1c = cvLoadImage(imageNameChar, 0);
	
    currFrameMaskTool_1 = cvLoadImage(imageNameChar1, 0);
    currFrameMaskTool_2 = cvLoadImage(imageNameChar2, 0);
}

void GWTracking::toolsDetection()
{
    Mat imgHSV;
    Mat imgThresholded;
    Mat imgOriginal = cvarrToMat(currFrame3c);

    cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    //inRange(imgHSV, Scalar(0, 130, 100), Scalar(150, 255, 255), imgThresholded); //Threshold the image
    inRange(imgHSV, Scalar(30, 100, 56), Scalar(179, 255, 255), imgThresholded); //Threshold the image
    //Bidan change to:
    //inRange(imgHSV, Scalar(0, 42, 33), Scalar(179, 255, 255), imgThresholded); //Threshold the image

    //morphological opening (removes small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    //morphological closing (removes small holes from the foreground)
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    for(int z=0; z < imgThresholded.cols; z++)
        for(int j=0; j < imgThresholded.rows; j++)
        {
            int value = imgThresholded.at<unsigned char>(j,z);
            if( value == 255 )
                ((uchar *)(currFrameMaskTool_1->imageData + j*currFrameMaskTool_1->widthStep))[z] = 255;
            else
                ((uchar *)(currFrameMaskTool_1->imageData + j*currFrameMaskTool_1->widthStep))[z] = 0;
        }
    //cvShowImage("tool",currFrameMaskTool_1);
    //cvWaitKey();
}

void GWTracking::linenessCurrentFrame()
{
	HessianAnalysisEigenvaluesStandard ( currFrame1c, sigmaHessian, kernelDimension, currLineness );
}

void GWTracking::binarizeCurrentLineness(IplImage *currBinaryFrame_out)
{
	Mat temp = cvarrToMat(currBinaryFrame_out);
	Mat currBinaryFrameMat( currFrame1c->width, currFrame1c->height, IPL_DEPTH_8U, cvScalar(0)); 

	//Binarized lineness image
	for (int i = roi.y; i < (roi.y +  roi.height); i++ )
		for (int j = roi.x; j < (roi.x + roi.width); j++ )
		{
			float valueCurrentPixel = ((float *)(currLineness->imageData + i*currLineness->widthStep))[j];
			if( valueCurrentPixel >= minLinenessValue)
				temp.at<uchar>(i,j) = 255;
		}

	thinning(temp, currBinaryFrameMat);

    //cvDilate(currFrameMaskTool_1, currFrameMaskTool_1, NULL, 8);
    //cvDilate(currFrameMaskTool_2, currFrameMaskTool_2, NULL, 8);

    cvDilate(currFrameMaskTool_1, currFrameMaskTool_1, NULL, 1);
    cvDilate(currFrameMaskTool_2, currFrameMaskTool_2, NULL, 1);

    unsigned char *input = (unsigned char*)(currBinaryFrameMat.data);
	for(int z=0; z < currBinaryFrameMat.cols; z++)
		for(int j=0; j < currBinaryFrameMat.rows; j++)
		{
			float value = input[currBinaryFrameMat.step * j + z];
            int valueTool_1 = ((char *)(currFrameMaskTool_1->imageData + j*currFrameMaskTool_1->widthStep))[z];
            //int valueTool_2 = ((char *)(currFrameMaskTool_2->imageData + j*currFrameMaskTool_2->widthStep))[z];
            //if( value == 255 && valueTool_1 == 0 && valueTool_2 == 0)
            if( value == 255 && valueTool_1 == 0 )
            //if( value == 255 )
				((uchar *)(currBinaryFrame_out->imageData + j*currBinaryFrame_out->widthStep))[z] = 255;	
			else
				((uchar *)(currBinaryFrame_out->imageData + j*currBinaryFrame_out->widthStep))[z] = 0;
        }

    //cvShowImage("tool",currBinaryFrame_out);
    //cvWaitKey();
	
	temp.release();
	currBinaryFrameMat.release();
}

void GWTracking::extractStraightSegmentsGT( std::vector<straightSegment> &listSegmentsGT_out )
{
	int currentIndexSegment = -1;
	extractStraightSegmentsFromAConnectedRegionGT( maxNumPixelPerSegment, slopeThreshold, prevGW, &currentIndexSegment, minSizeSegment, listSegmentsGT_out );

	//Refine the Segments in order to have the last straight segment with fixed lenght = TIP
	refineSegmentsTip( minSizeSegment, numPixelTip, listSegmentsGT_out);
	currentIndexSegment = 0;
}

void GWTracking::extractStraightSegments( IplImage *currBinaryFrame_in, std::vector<straightSegment> &listSegmentsFilteredOut_out, std::vector<straightSegment> &listSegments_out )
{
	int currentIndexSegment = 0;
    std::vector<vector<Point2d> > listBlobs;
	extractBlobs( minSizeSegment, maxDisplacementSegments, currBinaryFrame_in, prevGW, listBlobs, listSegmentsFilteredOut_out );

	for (unsigned int i=0; i < listBlobs.size(); i++)
	{
		std::vector<straightSegment> currentListSegments;
		extractStraightSegmentsFromAConnectedRegion( maxNumPixelPerSegment, slopeThreshold, currBinaryFrame_in, listBlobs[i], &currentIndexSegment, minSizeSegment, currentListSegments );

		for (unsigned int j=0; j < currentListSegments.size(); j++)
			listSegments_out.push_back(currentListSegments[j]);		
	}
}

void GWTracking::chooseSEGlets( std::vector<vector<matchingCost> > matchingCosts_in, std::vector<straightSegment> listSegmentsGT_in, std::vector<straightSegment> listSegments_in, std::vector<straightSegment> &SEGlets_out, std::vector<straightSegment> &listSegmentsFilteredOut_out )
{
	int numberOfCandidatesForSegment;
	
	//choose the SEGlets
	matchingCost cost;
	for(int t=0; t<listSegmentsGT_in.size(); t++)
	{
		std::vector<Point2d> currentListSegment;
		std::sort(matchingCosts_in[t].begin(), matchingCosts_in[t].end(), cost);

        //if( listSegmentsGT_in[t].points.size() < lengthSegmentForCandidates )
            //numberOfCandidatesForSegment = 1;
        //else if( listSegmentsGT_in[t].points.size() >= lengthSegmentForCandidates )
            //numberOfCandidatesForSegment = 2;

        //if( t == (listSegmentsGT_in.size() - 1) )
            //numberOfCandidatesForSegment = 3;

        numberOfCandidatesForSegment = 3;

		//Check in case there aren't enough candidate segments available for the current segment
		if( matchingCosts_in[t].size() < numberOfCandidatesForSegment )
			numberOfCandidatesForSegment = matchingCosts_in[t].size();

		indexTipSegment.clear();

		for(int z=0; z < numberOfCandidatesForSegment; z++)
		{
			bool found = false;
			for(int v=0; v < SEGlets_out.size(); v++)
				if((SEGlets_out[v].points[0] == listSegments_in[matchingCosts_in[t][z].index].points[0]) && 
					(SEGlets_out[v].points[SEGlets_out[v].points.size()-1] == listSegments_in[matchingCosts_in[t][z].index].points[listSegments_in[matchingCosts_in[t][z].index].points.size()-1]))
					found = true;

			if(!found)
			{
				SEGlets_out.push_back(listSegments_in[matchingCosts_in[t][z].index]);
				if(t == (listSegmentsGT_in.size() - 1))
					indexTipSegment.push_back(SEGlets_out.size()-1);
			}
		}

		//In case of the "base" of the GW -> Add the segment closest to the image border on the first position of the array
        if(t == 0)
        //if(false)
		{
            int bestIndex = -1;
			if(startingPointInstrument == 0)
			{
				int refValue = INT_MIN;			
				for(int v=0; v < SEGlets_out.size(); v++)
				{
					if(SEGlets_out[v].points[0].y > refValue)
					{
						refValue = SEGlets_out[v].points[0].y;
						bestIndex = v;
					}
					else if(SEGlets_out[v].points[SEGlets_out[v].points.size()-1].y > refValue)
					{
						refValue = SEGlets_out[v].points[SEGlets_out[v].points.size()-1].y;
						bestIndex = v;
					}
				}
			}
			else if(startingPointInstrument == 1)
			{
				int refValue = INT_MAX;			
				for(int v=0; v < SEGlets_out.size(); v++)
				{
					if(SEGlets_out[v].points[0].y < refValue)
					{
						refValue = SEGlets_out[v].points[0].y;
						bestIndex = v;
					}
					else if(SEGlets_out[v].points[SEGlets_out[v].points.size()-1].y < refValue)
					{
						refValue = SEGlets_out[v].points[SEGlets_out[v].points.size()-1].y;
						bestIndex = v;
					}
				}
			}	
			straightSegment tmp = SEGlets_out[bestIndex];
			SEGlets_out.erase(SEGlets_out.begin() + bestIndex);		
            SEGlets_out.insert (SEGlets_out.begin(), tmp);

			//Create a list of segments that could be used as "base" of the GW
			for( int c=numberOfCandidatesForSegment; c<matchingCosts_in[t].size(); c++ )
				possibleSegmentsBase.push_back(listSegments_in[matchingCosts_in[t][c].index]); 
		}
	}

	//Create a list of segments which are not SEGlets
	for(int i=0; i < listSegments_in.size(); i++)
	{
		bool found = false;
		straightSegment currentSegment = listSegments_in[i];
		for(int v=0; v < SEGlets_out.size(); v++)
			if(SEGlets_out[v].id == currentSegment.id)
			{
				found = true;
				break;
			}
		if(found == false)
			listSegmentsFilteredOut_out.push_back(currentSegment);
	}
}

void GWTracking::calculateMatchingCosts( std::vector<straightSegment> listSegmentsGT_in, std::vector<straightSegment> listSegments_in, std::vector<vector<matchingCost> > &matchingCosts_out)
{
	for(int i=0; i<listSegmentsGT_in.size(); i++)
	{
		std::vector<matchingCost> currentCostsSegment;
		matchingCosts_out.push_back(currentCostsSegment);

		for(int j=0; j<listSegments_in.size(); j++)
		{
			matchingCost currentCost;
			currentCost.index = j;

			//Calculate min distance from segmentGT to the current segment - Unsorted
			double dist1 = sqrt( pow(listSegmentsGT_in[i].points[(int)(listSegmentsGT_in[i].points.size()/2)].x - listSegments_in[j].points[(int)(listSegments_in[j].points.size()/2)].x,2) 
				+ pow(listSegmentsGT_in[i].points[(int)(listSegmentsGT_in[i].points.size()/2)].y - listSegments_in[j].points[(int)(listSegments_in[j].points.size()/2)].y,2) );

			double minDistance = dist1;

			double dist2 = sqrt( pow(listSegmentsGT_in[i].points[0].x - listSegments_in[j].points[0].x,2) 
				+ pow(listSegmentsGT_in[i].points[0].y - listSegments_in[j].points[0].y,2) );

			if(minDistance > dist2)
				minDistance = dist2;

			double dist3 = sqrt( pow(listSegmentsGT_in[i].points[listSegmentsGT_in[i].points.size()-1].x - listSegments_in[j].points[0].x,2) 
				+ pow(listSegmentsGT_in[i].points[listSegmentsGT_in[i].points.size()-1].y - listSegments_in[j].points[0].y,2) );

			if(minDistance > dist3)
				minDistance = dist3;

			double dist4 = sqrt( pow(listSegmentsGT_in[i].points[listSegmentsGT_in[i].points.size()-1].x - listSegments_in[j].points[(int)(listSegments_in[j].points.size()-1)].x,2) 
				+ pow(listSegmentsGT_in[i].points[listSegmentsGT_in[i].points.size()-1].y - listSegments_in[j].points[(int)(listSegments_in[j].points.size()-1)].y,2) );

			if(minDistance > dist4)
				minDistance = dist4;

			double dist5 = sqrt( pow(listSegmentsGT_in[i].points[0].x - listSegments_in[j].points[(int)(listSegments_in[j].points.size()-1)].x,2) 
				+ pow(listSegmentsGT_in[i].points[0].y - listSegments_in[j].points[(int)(listSegments_in[j].points.size()-1)].y,2) );

			if(minDistance > dist5)
				minDistance = dist5;
			
			//If the distance between the two segment is less then the threshold => Calculate the matching cost between the two segments
			if(minDistance < maxDisplacementSegments) 
			{
				calculateSingleCost( listSegmentsGT_in[i].points, listSegments_in[j].points, &currentCost);
				matchingCosts_out[i].push_back(currentCost);
			}
		}
	}
}

void GWTracking::calculateSingleCost( std::vector<Point2d> prevSegment_in, std::vector<Point2d> nextSegment_in, matchingCost *cost_out)
{
	float costDistance = 0;
	float costOrientation = 0;
	float costLength = 0;
	float costBG = 0;

	Point2d directionsPrev;
	Point2d directionsNext;

	//C_dist
	float distance = 0;
	calculateDistanceSegments(prevSegment_in, nextSegment_in, &distance);
	costDistance = distance / maxDisplacementSegments;	
	if(costDistance > 1) costDistance = 1;

	//C_phi
	float orientationPrev = 0; float orientationNext = 0;
	findOrientationCloudPoints( prevSegment_in, &directionsPrev, &orientationPrev );
	findOrientationCloudPoints( nextSegment_in, &directionsNext, &orientationNext );
	float orientation = fabs(orientationPrev - orientationNext);

	if(orientation > 90)
	{
		if( abs(orientationPrev) > abs(orientationNext) )
			if( orientationPrev < 0 )	orientationPrev = orientationPrev + 180;
			else						orientationPrev = orientationPrev - 180;
		else if( abs(orientationPrev) < abs(orientationNext) )
			if( orientationNext < 0 ) 	orientationNext = orientationNext + 180;
			else						orientationNext = orientationNext - 180;
		else
		{
			if( orientationPrev < 0 )		orientationPrev = orientationPrev + 180;
			else							orientationNext = orientationNext + 180;
		}

		orientation = fabs(orientationPrev - orientationNext);
	}

	costOrientation = orientation / maxChangeInOrientationSegments;
	
	//C_len
	float length;
	int difLength = nextSegment_in.size() - prevSegment_in.size();
	if(nextSegment_in.size() > prevSegment_in.size())
		length = 0.0; //length = abs(difLength)/(nextSegment_in.size());  
	else
		length = abs(difLength)*1.0/(prevSegment_in.size());
	costLength = length / (maxChangeInLengthSegments);
	if(costLength > 1) costLength = 1;

	//C_BG
	for(int j=0; j < nextSegment_in.size(); j++)
	{
		Point2d currentPoint = nextSegment_in[j];
		double currentBGvalue = ((float *)(backgroundMap->imageData + (int)currentPoint.y*backgroundMap->widthStep))[(int)currentPoint.x];
		currentBGvalue = currentBGvalue / ((indexFrame - indexFirstFrame)*1.0);
		costBG = costBG + currentBGvalue;
	}
	costBG = costBG / (nextSegment_in.size()*1.0);

	//Total Matching cost f_c Eq.1
	cost_out->cost = costDistance*weightDistance + costOrientation*weightOrientation + costLength*weightLength + costBG*weightBG;
}

void GWTracking::calculatePossiblePaths( std::vector<straightSegment> SEGlets_in, std::vector<Point2d> &firstSegment_out, std::vector<vector<Point2d> > &pointsHps_out )
{
    std::vector<vector<Point2d> > straightSegments;
	std::vector<int> initialHp;
	
	for(int i=0; i < SEGlets_in.size(); i++)
		straightSegments.push_back(SEGlets_in[i].points);

	//Sort the first segment to start with
	firstSegment_out = straightSegments[0];
	if(startingPointInstrument == 0)
	{
		if(firstSegment_out[0].y < firstSegment_out[firstSegment_out.size()-1].y)
			reverse(firstSegment_out.begin(), firstSegment_out.end());
	}
	else if(startingPointInstrument == 1)
	{
		if(firstSegment_out[0].y > firstSegment_out[firstSegment_out.size()-1].y)
		//if(firstSegment_out[0].x < firstSegment_out[firstSegment_out.size()-1].x)
			reverse(firstSegment_out.begin(), firstSegment_out.end());
	}

	//Calculate the possible paths between straight segments starting from the best candidate on the GW base
	straightSegments[0].clear();	
	initialHp.push_back(0);
	findSegmentConnections( straightSegments, firstSegment_out, 0, initialHp );

	//Sort the segments and create a list of points of each possible path = tracking hypothesis
	for(unsigned int i=0; i < hps.size(); i++)
	{
		std::vector<Point2d> currentPointsHp;
		for (unsigned int r=0; r < hps[i].size(); r++)
		{
			std::vector<Point2d> currentSegment = SEGlets_in[abs(hps[i][r])].points;

			if(hps[i][r] == 0)
			{
				if(startingPointInstrument == 0)
				{
					if(currentSegment[0].y < currentSegment[currentSegment.size()-1].y)
						reverse(currentSegment.begin(), currentSegment.end());
				}
				else if(startingPointInstrument == 1)
				{
					if(currentSegment[0].y > currentSegment[currentSegment.size()-1].y)
						reverse(currentSegment.begin(), currentSegment.end());
				}
			}
			else if(hps[i][r] < 0)
				reverse(currentSegment.begin(), currentSegment.end());

			for (unsigned int y=0; y < currentSegment.size(); y++)
				currentPointsHp.push_back(currentSegment[y]);
		}
		reverse(currentPointsHp.begin(),currentPointsHp.end());
		pointsHps_out.push_back(currentPointsHp);
	}
}

void GWTracking::addBaseGW( std::vector<Point2d> firstSegment_in, bool *foundAtLeast1_out, straightSegment *bestSegmentToAdd_out )
{
	double minDistance   = DBL_MAX;

	//Add a "base" segment to the GW if it's needed
	for(unsigned int i=0; i < possibleSegmentsBase.size(); i++)
	{
		straightSegment currentSegment = possibleSegmentsBase[i];
		double distance1 = sqrt( pow(firstSegment_in[0].x - currentSegment.points[0].x,2) + pow(firstSegment_in[0].y - currentSegment.points[0].y,2) );
		double distance2 = sqrt( pow(firstSegment_in[0].x - currentSegment.points[(int)(currentSegment.points.size()-1)].x,2) + pow(firstSegment_in[0].y - currentSegment.points[(int)(currentSegment.points.size()-1)].y,2) );
		double distance = 0;
		
		if(distance1 > distance2)
		{
			if(distance2 < maxDistance_Base)
			{
				float orientation1 = 0;
				float orientation2 = 0;
				Point2d direction1;
				Point2d direction2;
				
				findOrientationCloudPointsSorted( currentSegment.points, &direction1, &orientation1 );
				findOrientationCloudPointsSorted( firstSegment_in, &direction2, &orientation2 );

				reverse(currentSegment.points.begin(),currentSegment.points.end());

				float differentOrientation = abs(orientation1 - orientation2 );
				if(differentOrientation < maxOrientation_Base)
					if(minDistance > distance2)
					{
						*foundAtLeast1_out = true;
						*bestSegmentToAdd_out = currentSegment;
						minDistance = distance2;
					}
			}
		}
		else
		{
			if(distance1 < maxDistance_Base)
			{
				float orientation1 = 0;
				float orientation2 = 0;
				Point2d direction1;
				Point2d direction2;

				reverse(currentSegment.points.begin(),currentSegment.points.end());
				
				findOrientationCloudPointsSorted( currentSegment.points, &direction1, &orientation1 );
				findOrientationCloudPointsSorted( firstSegment_in, &direction2, &orientation2 );

				reverse(currentSegment.points.begin(),currentSegment.points.end());

				float differentOrientation = abs(orientation1 - orientation2 );
				if(differentOrientation < maxOrientation_Base)
					if(minDistance > distance1)
					{
						*foundAtLeast1_out = true;
						*bestSegmentToAdd_out = currentSegment;
						minDistance = distance1;
					}
			}
		}
	}	
}

void GWTracking::createCurvesFromHps( bool foundAtLeast1_in, std::vector<vector<Point2d> > pointsHps_in, straightSegment bestSegmentToAdd_in, std::vector<trackingHp> &trackingHps_out )
{
	//Create smooth curves from sparse points
	for(unsigned int i=0; i < pointsHps_in.size(); i++)
	{
		trackingHp currentTrackingHp;
		std::vector<Point2d> currentPointsHp;
		std::vector<int> currentIndexCurve;

		currentPointsHp = pointsHps_in[i];

		if(foundAtLeast1_in)
		{
			for(unsigned int j=0; j < bestSegmentToAdd_in.points.size(); j++)
				currentPointsHp.push_back(bestSegmentToAdd_in.points[j]);
		}

		createSmoothCurveFromPoints2( distanceNodes, currentPointsHp, i, currFrame3c, startingPointInstrument, currentIndexCurve );
		currentTrackingHp.id = i;
		currentTrackingHp.indexes = currentIndexCurve;
		currentTrackingHp.points = currentPointsHp;

		trackingHps_out.push_back(currentTrackingHp);
	}
}

int GWTracking::findSegmentConnections( std::vector<vector<Point2d> > availableSegments_in, std::vector<Point2d> currentSegment_in, int currentIndexNode_in, std::vector<int> currentHypothesis_in )
{
	std::vector<int> indexConnectedSegments;
    std::vector<vector<Point2d> > connectedSegments;
	std::vector<double> segmentDistances;

	for(int i=0; i < availableSegments_in.size(); i++)
		if( availableSegments_in[i].size() != 0)
		{
			int connected;
			double currentDistance = 0;			
			bool isTipSegment = false;

			for(int j=0; j < indexTipSegment.size(); j++)
				if(indexTipSegment[j] == i) isTipSegment = true;					

			if(isTipSegment)
				connected = areConnectedTipSegments(currentSegment_in, availableSegments_in[i], &currentDistance);
			else
				connected = areConnected(currentSegment_in, availableSegments_in[i], &currentDistance);
			
			if( connected != 0 )
			{
				if(connectedSegments.size() == 0)
				{
					indexConnectedSegments.push_back((i+1)*connected);
					connectedSegments.push_back(availableSegments_in[i]);
					segmentDistances.push_back(currentDistance);
				}
				else if(connectedSegments.size() == 1)
				{
					if(currentDistance > segmentDistances[0])
					{
						indexConnectedSegments.push_back((i+1)*connected);
						connectedSegments.push_back(availableSegments_in[i]);
						segmentDistances.push_back(currentDistance);
					}
					else
					{
						indexConnectedSegments.push_back((i+1)*connected);
						connectedSegments.push_back(availableSegments_in[i]);
						segmentDistances.push_back(currentDistance);

						reverse(indexConnectedSegments.begin(),indexConnectedSegments.end());
						reverse(connectedSegments.begin(),connectedSegments.end());
						reverse(segmentDistances.begin(),segmentDistances.end());
					}
				}
				else
				{
					int index = -1;
					for(int z=0; z < connectedSegments.size(); z++)
						if(currentDistance < segmentDistances[z])
						{
							index = z;
							break;
						}

					if(index != -1)
					{
						indexConnectedSegments.insert( indexConnectedSegments.begin() + index, ((i+1)*connected) );
						connectedSegments.insert( connectedSegments.begin() + index, availableSegments_in[i] );
						segmentDistances.insert( segmentDistances.begin() + index, currentDistance );
					}
					else
					{
						indexConnectedSegments.push_back((i+1)*connected);
						connectedSegments.push_back(availableSegments_in[i]);
						segmentDistances.push_back(currentDistance);
					}
				}
			}
		}

	// In case one segment is connected to more than one segment => Filter the segments that are connected to each other.
	// They will be added anyway on the next iteration. This speeds up the calculation and avoid redundant Hps.
	if(connectedSegments.size() > 1)
		filterConnectedSegments( indexConnectedSegments, connectedSegments, segmentDistances );
	
	//Recursive call to continue with the generation of the possible paths using the SEGlets
	for(int i=0; i < connectedSegments.size(); i++)
	{
		std::vector<Point2d> currentSegment = availableSegments_in[abs(indexConnectedSegments[i])-1];
        std::vector<vector<Point2d> > availableSegmentsTemp = availableSegments_in;
		availableSegmentsTemp[abs(indexConnectedSegments[i]) - 1].clear();
		std::vector<int> currentHypothesisUpdated = currentHypothesis_in;

		bool found = false;
		if(i != 0)
			for(int y=0; (y < hps.size()) && (found==false); y++)
				for(int z=0; z < (hps[y].size()) && (found==false); z++)
					if(abs(hps[y][z]) == (abs(indexConnectedSegments[i])-1))
						found = true;

		if(found == false)
		{
			if(indexConnectedSegments[i] < 0)
			{
				reverse(currentSegment.begin(),currentSegment.end());
				currentHypothesisUpdated.push_back((abs(indexConnectedSegments[i])-1) * (-1));
			}
			else
				currentHypothesisUpdated.push_back((abs(indexConnectedSegments[i])-1) * (1));		

			findSegmentConnections( availableSegmentsTemp, currentSegment, (abs(indexConnectedSegments[i])-1), currentHypothesisUpdated );
		}
	}

	if(connectedSegments.size() == 0)
		hps.push_back(currentHypothesis_in);

	return 0;
}

int GWTracking::areConnectedTipSegments(std::vector<Point2d> currentSegment_in, std::vector<Point2d> possibleMatch_in, double *distanceSegments_out )
{
	float endPoint1, endPoint2;

	//Calculate end-point distance between the two segments
	endPoint1 = sqrt( pow(currentSegment_in[currentSegment_in.size()-1].x - possibleMatch_in[0].x,2) + pow(currentSegment_in[currentSegment_in.size()-1].y - possibleMatch_in[0].y,2) );
	endPoint2 = sqrt( pow(currentSegment_in[currentSegment_in.size()-1].x - possibleMatch_in[possibleMatch_in.size()-1].x,2) + pow(currentSegment_in[currentSegment_in.size()-1].y - possibleMatch_in[possibleMatch_in.size()-1].y,2) );
	
	if(endPoint1 > endPoint2)
		if(endPoint2 < distanceThreshold_Tip)
		{
			*distanceSegments_out = endPoint2;

			float orientationPrev = 0;
			float orientationNext = 0;
			float orientationCandidate = 0;

			Point2d directionsPrev;
			Point2d directionsNext;
			Point2d directionsCandidate;

			std::vector<Point2d> segmentsNext_in;
			std::vector<Point2d> possibleMatchReverse;

			possibleMatchReverse = possibleMatch_in;
			reverse(possibleMatchReverse.begin(), possibleMatchReverse.end());

			//It is the missing segment which connects the two current segments
			segmentsNext_in.push_back(currentSegment_in[currentSegment_in.size()-1]);
			segmentsNext_in.push_back(possibleMatch_in[possibleMatch_in.size()-1]);

			findOrientationCloudPointsSorted( currentSegment_in, &directionsPrev, &orientationPrev );
			findOrientationCloudPointsSorted( segmentsNext_in, &directionsNext, &orientationNext );
			findOrientationCloudPointsSorted( possibleMatchReverse, &directionsCandidate, &orientationCandidate );

			float orientation1 = abs(orientationPrev - orientationNext);
			float orientation2 = abs(orientationCandidate - orientationNext);
			float orientationMinDif;
			float orientationMaxDif;

			if( orientation1 > orientation2 )
			{
				orientationMinDif = orientation2;
				orientationMaxDif = orientation1;
			}
			else
			{
				orientationMinDif = orientation1;
				orientationMaxDif = orientation2;
			}

			if(endPoint2 < minDistanceThreshold_Tip)
			{
				orientationMinDif = 0;
				orientationMaxDif = 0;
			}

			if( ( orientationMinDif < angleThreshold_Tip ) && ( orientationMaxDif < angleThreshold_Tip*2 ) )
				return -1;
			else
				return 0;
		}
		else
			return 0;
	else
		if(endPoint1 < distanceThreshold_Tip)
		{
			*distanceSegments_out = endPoint1;

			float orientationPrev = 0;
			float orientationNext = 0;
			float orientationCandidate = 0;

			Point2d directionsPrev;
			Point2d directionsNext;
			Point2d directionsCandidate;

			std::vector<Point2d> segmentsNext_in;

			//It is the missing segment which connects the two current segments
			segmentsNext_in.push_back(currentSegment_in[currentSegment_in.size()-1]);
			segmentsNext_in.push_back(possibleMatch_in[0]);	

			findOrientationCloudPointsSorted( currentSegment_in, &directionsPrev, &orientationPrev );
			findOrientationCloudPointsSorted( segmentsNext_in, &directionsNext, &orientationNext );
			findOrientationCloudPointsSorted( possibleMatch_in, &directionsCandidate, &orientationCandidate );

			float orientation1 = abs(orientationPrev - orientationNext);
			float orientation2 = abs(orientationCandidate - orientationNext);
			float orientationMinDif;
			float orientationMaxDif;

			if( orientation1 > orientation2 )
			{
				orientationMinDif = orientation2;
				orientationMaxDif = orientation1;
			}
			else
			{
				orientationMinDif = orientation1;
				orientationMaxDif = orientation2;
			}

			if(endPoint1 < minDistanceThreshold_Tip)
			{
				orientationMinDif = 0;
				orientationMaxDif = 0;
			}

			if( ( orientationMinDif < angleThreshold_Tip ) && ( orientationMaxDif < angleThreshold_Tip*2 ) )
				return 1;
			else
				return 0;
		}
		else
			return 0;
}

int GWTracking::areConnected(std::vector<Point2d> currentSegment_in, std::vector<Point2d> possibleMatch_in, double *distanceSegments_out )
{
	float endPoint1, endPoint2;

	//Calculate end-point distance between the two segments
	endPoint1 = sqrt( pow(currentSegment_in[currentSegment_in.size()-1].x - possibleMatch_in[0].x,2) + pow(currentSegment_in[currentSegment_in.size()-1].y - possibleMatch_in[0].y,2) );
	endPoint2 = sqrt( pow(currentSegment_in[currentSegment_in.size()-1].x - possibleMatch_in[possibleMatch_in.size()-1].x,2) + pow(currentSegment_in[currentSegment_in.size()-1].y - possibleMatch_in[possibleMatch_in.size()-1].y,2) );
	
	if(endPoint1 > endPoint2)
		if(endPoint2 < distanceThreshold_Seg)
		{
			*distanceSegments_out = endPoint2;

			float orientationPrev = 0;
			float orientationNext = 0;
			float orientationCandidate = 0;

			Point2d directionsPrev;
			Point2d directionsNext;
			Point2d directionsCandidate;

			std::vector<Point2d> segmentsNext_in;
			std::vector<Point2d> possibleMatchReverse;

			possibleMatchReverse = possibleMatch_in;
			reverse(possibleMatchReverse.begin(), possibleMatchReverse.end());

			//It is the missing segment which connects the two current segments
			segmentsNext_in.push_back(currentSegment_in[currentSegment_in.size()-1]);
			segmentsNext_in.push_back(possibleMatch_in[possibleMatch_in.size()-1]);

			findOrientationCloudPointsSorted( currentSegment_in, &directionsPrev, &orientationPrev );
			findOrientationCloudPointsSorted( segmentsNext_in, &directionsNext, &orientationNext );
			findOrientationCloudPointsSorted( possibleMatchReverse, &directionsCandidate, &orientationCandidate );

			float orientation1 = abs(orientationPrev - orientationNext);
			float orientation2 = abs(orientationCandidate - orientationNext);
			float orientationMinDif;
			float orientationMaxDif;

			if( orientation1 > orientation2 )
			{
				orientationMinDif = orientation2;
				orientationMaxDif = orientation1;
			}
			else
			{
				orientationMinDif = orientation1;
				orientationMaxDif = orientation2;
			}

			if(endPoint2 < minDistanceThreshold_Seg)
			{
				orientationMinDif = 0;
				orientationMaxDif = 0;
			}

			if( ( orientationMinDif < angleThreshold_Seg ) && ( orientationMaxDif < angleThreshold_Seg*2 ) )
				return -1;
			else
				return 0;
		}
		else
			return 0;
	else
		if(endPoint1 < distanceThreshold_Seg)
		{
			*distanceSegments_out = endPoint1;

			float orientationPrev = 0;
			float orientationNext = 0;
			float orientationCandidate = 0;

			Point2d directionsPrev;
			Point2d directionsNext;
			Point2d directionsCandidate;

			std::vector<Point2d> segmentsNext_in;

			//It is the missing segment which connects the two current segments
			segmentsNext_in.push_back(currentSegment_in[currentSegment_in.size()-1]);
			segmentsNext_in.push_back(possibleMatch_in[0]);	

			findOrientationCloudPointsSorted( currentSegment_in, &directionsPrev, &orientationPrev );
			findOrientationCloudPointsSorted( segmentsNext_in, &directionsNext, &orientationNext );
			findOrientationCloudPointsSorted( possibleMatch_in, &directionsCandidate, &orientationCandidate );

			float orientation1 = abs(orientationPrev - orientationNext);
			float orientation2 = abs(orientationCandidate - orientationNext);
			float orientationMinDif;
			float orientationMaxDif;

			if( orientation1 > orientation2 )
			{
				orientationMinDif = orientation2;
				orientationMaxDif = orientation1;
			}
			else
			{
				orientationMinDif = orientation1;
				orientationMaxDif = orientation2;
			}

			if(endPoint1 < minDistanceThreshold_Seg)
			{
				orientationMinDif = 0;
				orientationMaxDif = 0;
			}

			if( ( orientationMinDif < angleThreshold_Seg ) && ( orientationMaxDif < angleThreshold_Seg*2 ) )
				return 1;
			else
				return 0;
		}
		else
			return 0;
}

void GWTracking::filterConnectedSegments( std::vector<int> &indexConnectedSegments_inout, std::vector<vector<Point2d> > &connectedSegments_inout, std::vector<double> &segmentDistances_inout )
{
	std::vector<int> indexConnectedSegmentsTemp;
    std::vector<vector<Point2d> > connectedSegmentsTemp;
	std::vector<double> segmentDistancesTemp;

	indexConnectedSegmentsTemp = indexConnectedSegments_inout;
	connectedSegmentsTemp = connectedSegments_inout;
	segmentDistancesTemp = segmentDistances_inout;

	indexConnectedSegments_inout.clear();
	connectedSegments_inout.clear();
	segmentDistances_inout.clear();

	while(!connectedSegmentsTemp.empty())
	{
		std::vector<Point2d> currentSegment;

		currentSegment = connectedSegmentsTemp[0];
		if(indexConnectedSegmentsTemp[0] < 0)
			reverse(currentSegment.begin(), currentSegment.end());

		indexConnectedSegments_inout.push_back(indexConnectedSegmentsTemp[0]);
		connectedSegments_inout.push_back(connectedSegmentsTemp[0]);
		segmentDistances_inout.push_back(segmentDistancesTemp[0]);

		indexConnectedSegmentsTemp.erase(indexConnectedSegmentsTemp.begin() + 0);
		connectedSegmentsTemp.erase(connectedSegmentsTemp.begin() + 0);
		segmentDistancesTemp.erase(segmentDistancesTemp.begin() + 0);

		int numberOfElements = connectedSegmentsTemp.size() - 1;
		if(!connectedSegmentsTemp.empty())
		{
			for(int i=numberOfElements; i >= 0; i--)
			{
				bool isTipSegment = false;

				double currentDistance = 0;				

				int indexCurrentSegment = abs(indexConnectedSegmentsTemp[i]) - 1;
				int connected;

				for(int j=0; j < indexTipSegment.size(); j++)
					if(indexTipSegment[j] == indexCurrentSegment) isTipSegment = true;					

				if(isTipSegment)
					connected = areConnectedTipSegments(currentSegment, connectedSegmentsTemp[i], &currentDistance);
				else
					connected = areConnected(currentSegment, connectedSegmentsTemp[i], &currentDistance);
				
				if( connected != 0 )
				{
					indexConnectedSegmentsTemp.erase(indexConnectedSegmentsTemp.begin() + i);
					connectedSegmentsTemp.erase(connectedSegmentsTemp.begin() + i);
					segmentDistancesTemp.erase(segmentDistancesTemp.begin() + i);
				}
			}
		}
	}
}

void GWTracking::calculateBackgroundMap( int indexFrame_in, std::vector<Point2d> centrelineGT_in, std::vector<straightSegment> listSegments_in )
{
	/*float lost = 0;
	for (int i = 0; i < backgroundMap->height; i++ )
		for (int j = 0; j < backgroundMap->width; j++ )
		{
			float currentPixelValue = ((float *)(backgroundMap->imageData + i*backgroundMap->widthStep))[j];
			((float *)(backgroundMap->imageData + i*backgroundMap->widthStep))[j] =  currentPixelValue - lost;
		}*/

	for (int i = 0; i < listSegments_in.size(); i++ )
		for (int j = 0; j < listSegments_in[i].points.size(); j++ )
		{
			float currentPixelValue = ((float *)(backgroundMap->imageData + (int)listSegments_in[i].points[j].y*backgroundMap->widthStep))[(int)listSegments_in[i].points[j].x];
			((float *)(backgroundMap->imageData + (int)listSegments_in[i].points[j].y*backgroundMap->widthStep))[(int)listSegments_in[i].points[j].x] =  currentPixelValue + gain;
		}

	for (int i = 0; i < centrelineGT_in.size(); i++ )
	{
		float currentPixelValue = ((float *)(backgroundMap->imageData + (int)centrelineGT_in[i].y*backgroundMap->widthStep))[(int)centrelineGT_in[i].x];
		if(currentPixelValue != 0)
			((float *)(backgroundMap->imageData + (int)centrelineGT_in[i].y*backgroundMap->widthStep))[(int)centrelineGT_in[i].x] = currentPixelValue - 1;
	}
}

void GWTracking::calculateGWModel( std::vector<Point2d> centrelineGT_in, IplImage *prevImage_in, IplImage *prevImageColor_in, trackingHp *model_out )
{
	//Create Model from Previous tracking	
	std::vector<Point2d> modelPoints;
	std::vector<int> indexCurveModel;	

	modelPoints = centrelineGT_in;
	
	createSmoothCurveFromPoints2( distanceNodes, modelPoints, 1000, prevImage_in, startingPointInstrument, indexCurveModel);
	
	model_out->id = -1;
	model_out->points = modelPoints;
	model_out->indexes = indexCurveModel;

	curvePatternCurrentHp.clear(); normalPattern1CurrentHp.clear(); normalPattern2CurrentHp.clear();
	calculateLocalBinaryPatterns( indexFrame, *model_out, prevImage_in );

	if( verbose != 0 )
		printLocalBinaryPatterns( prevImageColor_in, 0, seqName, indexFrame, *model_out, curvePatternCurrentHp, normalPattern1CurrentHp, normalPattern2CurrentHp );
}

void GWTracking::calculateLocalBinaryPatterns( int indexFrame_in, trackingHp &currentHp_in, IplImage *currentImage_in )
{
	//Image Based Intensity 
	for (int i = 0; i < currentHp_in.indexes.size(); i++ )
	{
		double currentCurvePattern = 0;
		double normalPattern1 = 0;
		double normalPattern2 = 0;
		std::vector<Point2d> currentCurvePoints;

		if(i == 0)
		{
			//Calculate currentCurvePattern
			for (int j = 0; j < lengthSegmentForIntensityMeasure_GW; j++ )
			{
				double currentCurveIntensity;
				Point2d currentPoint;
				currentPoint.x = currentHp_in.points[j].x;
				currentPoint.y = currentHp_in.points[j].y;
				currentCurvePoints.push_back(currentPoint);
				currentCurveIntensity = ((uchar *)(currentImage_in->imageData + (int)currentPoint.y*currentImage_in->widthStep))[(int)currentPoint.x];
				currentCurvePattern = currentCurvePattern + currentCurveIntensity;
				curvePatternCurrentHp.push_back(currentPoint);
			}
			currentCurvePattern = (currentCurvePattern/lengthSegmentForIntensityMeasure_GW);
			currentHp_in.curvePattern.push_back(currentCurvePattern);

			//Calculate calculateNormalPatterns
			calculateNormalPattern( indexFrame_in, currentCurvePoints[0], currentCurvePoints, currentImage_in, &normalPattern1, &normalPattern2 );
			currentHp_in.normalPattern1.push_back(normalPattern1);
			currentHp_in.normalPattern2.push_back(normalPattern2);
		}
		else if(i != currentHp_in.indexes.size() - 1)
		{
			//Calculate currentCurvePattern
			for (int j = ((-1)*(lengthSegmentForIntensityMeasure_GW - 1))/2; j < (lengthSegmentForIntensityMeasure_GW - 1)/2; j++ )
			{
				double currentCurveIntensity;
				Point2d currentPoint;
				currentPoint.x = currentHp_in.points[currentHp_in.indexes[i] + j].x;
				currentPoint.y = currentHp_in.points[currentHp_in.indexes[i] + j].y;
				currentCurvePoints.push_back(currentPoint);
				currentCurveIntensity = ((uchar *)(currentImage_in->imageData + (int)currentPoint.y*currentImage_in->widthStep))[(int)currentPoint.x];
				currentCurvePattern = currentCurvePattern + currentCurveIntensity;
				curvePatternCurrentHp.push_back(currentPoint);
			}
			currentCurvePattern = (currentCurvePattern/lengthSegmentForIntensityMeasure_GW);
			currentHp_in.curvePattern.push_back(currentCurvePattern);

			//Calculate calculateNormalPatterns
			calculateNormalPattern( indexFrame_in, currentCurvePoints[(int)(currentCurvePoints.size()/2)], currentCurvePoints, currentImage_in, &normalPattern1, &normalPattern2 );
			currentHp_in.normalPattern1.push_back(normalPattern1);
			currentHp_in.normalPattern2.push_back(normalPattern2);
		}
		else
		{
			//Calculate currentCurvePattern
			for (int j = 0; j < lengthSegmentForIntensityMeasure_GW; j++ )
			{
				double currentCurveIntensity;
				Point2d currentPoint;
				currentPoint.x = currentHp_in.points[currentHp_in.indexes[i] - j].x;
				currentPoint.y = currentHp_in.points[currentHp_in.indexes[i] - j].y;
				currentCurvePoints.push_back(currentPoint);
				currentCurveIntensity = ((uchar *)(currentImage_in->imageData + (int)currentPoint.y*currentImage_in->widthStep))[(int)currentPoint.x];
				currentCurvePattern = currentCurvePattern + currentCurveIntensity;
				curvePatternCurrentHp.push_back(currentPoint);
			}
			currentCurvePattern = (currentCurvePattern/lengthSegmentForIntensityMeasure_GW);
			currentHp_in.curvePattern.push_back(currentCurvePattern);
			
			reverse(currentCurvePoints.begin(),currentCurvePoints.end());
			//Calculate calculateNormalPatterns
			calculateNormalPattern( indexFrame_in, currentCurvePoints[currentCurvePoints.size()-1], currentCurvePoints, currentImage_in, &normalPattern1, &normalPattern2 );
			currentHp_in.normalPattern1.push_back(normalPattern1);
			currentHp_in.normalPattern2.push_back(normalPattern2);
		}

		//CALCULATE PATTERNS: SLPB_Pattern, Spline_Pattern and Bar_Pattern
		currentHp_in.Spline_Pattern.push_back(currentCurvePattern);
		
		currentHp_in.Bar_Pattern.push_back(normalPattern1); 
		currentHp_in.Bar_Pattern.push_back(currentCurvePattern); 
		currentHp_in.Bar_Pattern.push_back(normalPattern2);

		int a1, a2, a3;
		if( normalPattern1 > normalPattern2 )		a1 = 1;
		else										a1 = 0;

		if( normalPattern2 > currentCurvePattern )	a2 = 1;
		else										a2 = 0;

		if( normalPattern1 > currentCurvePattern )	a3 = 1;
		else										a3 = 0;

		currentHp_in.SLPB_Pattern.push_back(a1);
		currentHp_in.SLPB_Pattern.push_back(a2);
		currentHp_in.SLPB_Pattern.push_back(a3);
	}
}

void GWTracking::calculateNormalPattern( int indexFrame_in, Point2d startingPoint_in, std::vector<Point2d> curve_in, IplImage *currentImage_in, double *normalPattern1_out, double *normalPattern2_out )
{
	int sign1;
	int sign2;

	CvPoint pt1;
	float line[4];
	int numInterpolatedPoints = curve_in.size();

	CvPoint* points = (CvPoint*)malloc( numInterpolatedPoints * sizeof(points[0]));
	CvMat pointMat = cvMat( 1, numInterpolatedPoints, CV_32SC2, points );

	for(int z=0; z < numInterpolatedPoints; z++)
	{
		points[z].x = curve_in[z].x;
		points[z].y = curve_in[z].y;
	}

	if((curve_in[numInterpolatedPoints-1].x - curve_in[0].x) >= 0)
		sign1 = 1;
	else
		sign1 = -1;

	if((curve_in[numInterpolatedPoints-1].y - curve_in[0].y) >= 0)
		sign2 = 1;
	else
		sign2 = -1;

	cvFitLine( &pointMat, CV_DIST_L1, 1, 0.001, 0.001, line );
	line[0] = abs(line[0])*sign1;
	line[1] = abs(line[1])*sign2;

	float xCurveUnit = line[0]/sqrt(pow(line[0],2) + pow(line[1],2));
	float yCurveUnit = line[1]/sqrt(pow(line[0],2) + pow(line[1],2));

	float xNormalPattern1Unit = xCurveUnit*cos(-1.0*(M_PI/2.0)) - yCurveUnit*sin(-1.0*(M_PI/2.0));
	float yNormalPattern1Unit = xCurveUnit*sin(-1.0*(M_PI/2.0)) + yCurveUnit*cos(-1.0*(M_PI/2.0));

	float xNormalPattern2Unit = xCurveUnit*cos(M_PI/2.0) - yCurveUnit*sin(M_PI/2.0);
	float yNormalPattern2Unit = xCurveUnit*sin(M_PI/2.0) + yCurveUnit*cos(M_PI/2.0);

	int realLenghtSegmentPattern1 = 0;
	int realLenghtSegmentPattern2 = 0;
	for(int z=0; z < lengthSegmentForIntensityMeasure_Normal; z++)
	{
		Point2d currenNormalPattern1Point;
		Point2d currenNormalPattern2Point;
		
		currenNormalPattern1Point.x = (int)(startingPoint_in.x + xNormalPattern1Unit*(distanceNormalPatterns+z));
		currenNormalPattern1Point.y = (int)(startingPoint_in.y + yNormalPattern1Unit*(distanceNormalPatterns+z));
		currenNormalPattern2Point.x = (int)(startingPoint_in.x + xNormalPattern2Unit*(distanceNormalPatterns+z));
		currenNormalPattern2Point.y = (int)(startingPoint_in.y + yNormalPattern2Unit*(distanceNormalPatterns+z));

		if(	(currenNormalPattern1Point.x <= currentImage_in->width) && (currenNormalPattern1Point.x >= 0) && 
			(currenNormalPattern1Point.y <= currentImage_in->height) && (currenNormalPattern1Point.y >= 0)	)
		{
			double currentnormalPattern1Intensity = ((uchar *)(currentImage_in->imageData + (int)currenNormalPattern1Point.y*currentImage_in->widthStep))[(int)currenNormalPattern1Point.x];
			*normalPattern1_out = *normalPattern1_out + currentnormalPattern1Intensity;
			normalPattern1CurrentHp.push_back(currenNormalPattern1Point);
			realLenghtSegmentPattern1++;
		}

		if(	(currenNormalPattern2Point.x <= currentImage_in->width) && (currenNormalPattern2Point.x >= 0) && 
			(currenNormalPattern2Point.y <= currentImage_in->height) && (currenNormalPattern2Point.y >= 0)	)
		{
			double currentnormalPattern2Intensity = ((uchar *)(currentImage_in->imageData + (int)currenNormalPattern2Point.y*currentImage_in->widthStep))[(int)currenNormalPattern2Point.x];
			*normalPattern2_out = *normalPattern2_out + currentnormalPattern2Intensity;
			normalPattern2CurrentHp.push_back(currenNormalPattern2Point);
			realLenghtSegmentPattern2++;
		}
	}

	*normalPattern1_out = *normalPattern1_out/realLenghtSegmentPattern1;
	*normalPattern2_out = *normalPattern2_out/realLenghtSegmentPattern2;
}

void GWTracking::updateLinenessModel( int indexHp_in, trackingHp bestHP_in, trackingHp model_in, IplImage *currentImageLineness_in, IplImage *prevImageLineness_in )
{
	if(indexHp_in == 0)
	{
		double meanLineness = 0;
		std::vector<double> listMeanLineness;

		meanLinenessModel = 0;
		sigmaLinenessModel = 0;

		for (int i = 0; i < model_in.points.size(); i++ )
		{
			double currentLineness = 0;
			double currentVarianceHp = 0;
		
			Point2d currentPoint = model_in.points[i];
			currentLineness = ((float *)(prevImageLineness_in->imageData + (int)currentPoint.y*prevImageLineness_in->widthStep))[(int)currentPoint.x];
		
			meanLineness = meanLineness + currentLineness;
			listMeanLineness.push_back(currentLineness);
		}

		meanLineness = meanLineness / ( 1.0*model_in.points.size() );
		meanLinenessModel = meanLineness;

		for (int i = 0; i < model_in.points.size(); i++ )
			sigmaLinenessModel = sigmaLinenessModel + pow(( listMeanLineness[i] - meanLinenessModel),2);

		sigmaLinenessModel = sqrt(sigmaLinenessModel / model_in.points.size());
	}

	else
	{
		double meanLineness  = 0.0;
		double sigmaLineness = 0.0;
		std::vector<double> listMeanLineness;

		for (int i = 0; i < bestHP_in.points.size(); i++ )
		{
			double currentLineness	 = 0.0;
			double currentVarianceHp = 0.0;
		
			Point2d currentPoint = bestHP_in.points[i];
			currentLineness = ((float *)(currentImageLineness_in->imageData + (int)currentPoint.y*currentImageLineness_in->widthStep))[(int)currentPoint.x];
		
			meanLineness = meanLineness + currentLineness;
			listMeanLineness.push_back(currentLineness);
		}
	
		meanLineness = meanLineness / (1.0*bestHP_in.points.size());
		meanLinenessModel = ((1.0-memoryModel)*meanLinenessModel) + (memoryModel * meanLineness);

		for (int i = 0; i < bestHP_in.points.size(); i++ )
			sigmaLineness = sigmaLineness + pow( (listMeanLineness[i] - meanLineness), 2 );

		sigmaLineness = sqrt( sigmaLineness / (1.0*bestHP_in.points.size()) );

		sigmaLinenessModel = ((1.0-memoryModel)*sigmaLinenessModel) + ( memoryModel * sigmaLineness);
	}
}

void GWTracking::evaluateLikelihoodCurrentHp( ofstream &myfile_in, int indexHp_in, trackingHp currentHp_in, trackingHp model_in, IplImage *prevImage_in, IplImage *currentImage_in, IplImage *prevImageLineness_in, IplImage *currentImageLineness_in, double *energyHp_out )
{
	double priorShape		= 0.0;
	double intensityPatternMeasure = 0.0;
	double linenessMeasure	= 0.0;

	//Prior
	calculatePriorCurrentHp(currentHp_in, model_in, &priorShape);

	//Intensity Pattern Measurment
	calculateIntensityPatternCurrentHp( currentHp_in, model_in, &intensityPatternMeasure );

	//Lineness Measurment
	calculateLinenessMeasureCurrentHp( indexHp_in, currentHp_in, currentImageLineness_in, &linenessMeasure );

	//Calculate Total Energy
	*energyHp_out = priorShape * ( weightMeasures[0]*intensityPatternMeasure  + weightMeasures[1]*linenessMeasure);
	
	if( verbose != 0 )
	{
		myfile_in << "HYPOTHESIS #" << indexHp_in << "\n";
		myfile_in << "	PRIOR					" << priorShape << "\n";
		myfile_in << "	INTENSITY PATTERN		" << intensityPatternMeasure << "\n";
		myfile_in << "	LINENESS				" << linenessMeasure << "\n";
		myfile_in << "	TOTAL SCORE				" << *energyHp_out << "\n";
		myfile_in << "\n";
		myfile_in << "	MEAN LINENESS MODEL		" << meanLinenessModel << "\n";
		myfile_in << "	SIGMA LINENESS MODEL	" << sigmaLinenessModel << "\n";
		myfile_in << " ------------------------------------------------- \n";
	}
	
}

void GWTracking::calculatePriorCurrentHp( trackingHp currentHp_in, trackingHp model_in, double *prior_out )
{
	double priorShape = 0;
	double sumDistancesHpToModel = 0;
	double sumDistancesModelToHP = 0;
	double totalDistance = 0;

	//Prior
	for (int i = 0; i < currentHp_in.indexes.size(); i++ )
	{
		Point2d currentPoint = currentHp_in.points[currentHp_in.indexes[i]];
		double currentDistance = DBL_MAX;
		
		for (int j = 0; j < model_in.points.size(); j++ )
		{
			double candidateDistance = sqrt( pow(currentPoint.x - model_in.points[j].x,2) + pow(currentPoint.y - model_in.points[j].y,2) );
			if(candidateDistance < currentDistance)
				currentDistance = candidateDistance;
		}
		sumDistancesHpToModel = sumDistancesHpToModel + currentDistance;
	}

	sumDistancesHpToModel = sumDistancesHpToModel/currentHp_in.indexes.size();

	for (int i = 0; i < model_in.indexes.size(); i++ )
	{
		Point2d currentPoint = model_in.points[model_in.indexes[i]];
		double currentDistance = DBL_MAX;
		
		for (int j = 0; j < currentHp_in.points.size(); j++ )
		{
			double candidateDistance = sqrt( pow(currentPoint.x - currentHp_in.points[j].x,2) + pow(currentPoint.y - currentHp_in.points[j].y,2) );
			if(candidateDistance < currentDistance)
				currentDistance = candidateDistance;
		}
		sumDistancesModelToHP = sumDistancesModelToHP + currentDistance;
	}

	sumDistancesModelToHP = sumDistancesModelToHP/model_in.indexes.size();

	totalDistance = sumDistancesHpToModel + sumDistancesModelToHP; 

	priorShape = (1/(sqrt(2*M_PI)*sigmaPrior))*exp(-1*(pow(totalDistance,2)/(2*pow(sigmaPrior,2))));
	*prior_out = priorShape;
}

void GWTracking::calculateIntensityPatternCurrentHp( trackingHp currentHp_in, trackingHp model_in, double *internsityPatternMeasure_out )
{
	std::vector<double> currentPatternHp	= currentHp_in.SLPB_Pattern;
	std::vector<double> currentPatternModel = model_in.SLPB_Pattern;

	int minCardinality;
	if( currentPatternHp.size() > currentPatternModel.size() )
		minCardinality = currentPatternModel.size();
	else
		minCardinality = currentPatternHp.size();

	double modulePatternHp		= 0.0;
	double modulePatternModel	= 0.0;
	double crossProductPatterns = 0.0;

	for(int i=0; i<minCardinality; i++ )
		modulePatternHp = modulePatternHp + pow( currentPatternHp[i], 2);
	modulePatternHp = sqrt(modulePatternHp);

	for(int i=0; i<minCardinality; i++ )
		modulePatternModel = modulePatternModel + pow( currentPatternModel[i], 2);
	modulePatternModel = sqrt(modulePatternModel);

	for(int i=0; i<minCardinality; i++ )
		crossProductPatterns = crossProductPatterns + currentPatternHp[i]*currentPatternModel[i];	

	*internsityPatternMeasure_out = (fabs(crossProductPatterns)/(modulePatternHp*modulePatternModel));
}

void GWTracking::calculateLinenessMeasureCurrentHp( int indexHp_in, trackingHp currentHp_in, IplImage *currentImageLineness_in, double *linenessMeasure_out )
{
	for (int i = 0; i < currentHp_in.indexes.size(); i++ )
	{
		double currentLineness = 0;
		double currentVarianceHp = 0;
		
		Point2d currentPoint = currentHp_in.points[currentHp_in.indexes[i]];
		currentLineness = ((float *)(currentImageLineness_in->imageData + (int)currentPoint.y*currentImageLineness_in->widthStep))[(int)currentPoint.x];
		
		*linenessMeasure_out = *linenessMeasure_out + (1.0/(sigmaLinenessModel*sqrt(M_PI*2)))*exp( ( (-1) * pow((currentLineness - meanLinenessModel),2) ) / ( 2 * pow(sigmaLinenessModel,2) ) );
	}

	*linenessMeasure_out = *linenessMeasure_out / (1.0 * currentHp_in.indexes.size());
}
