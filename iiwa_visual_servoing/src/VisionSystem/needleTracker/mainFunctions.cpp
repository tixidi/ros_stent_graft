#include "mainFunctions.h"

void calculateKernel( float sigma_in, int kernelDimension_in, int typeKernel_in,  CvMat * Kernel_out)
{
	for(int y=0; y < kernelDimension_in; y++)
		for(int x=0; x < kernelDimension_in; x++)
		{
			float elementValue;

			int newX = (x - (kernelDimension_in - 1)/2);
			int newY = (y - (kernelDimension_in - 1)/2);

			float exponent  = (pow((float) newX,2) + pow((float) newY,2)) / (2*pow(sigma_in, 2));

			//XX
			if(typeKernel_in == 1)
			{
				float base = (pow((float)newX, 2) - pow(sigma_in, 2)) / ( 2*pow(sigma_in, 6)*M_PI );
				elementValue = base * exp((-1)*exponent);
			}

			//XY
			if(typeKernel_in == 2)
			{		
				float base = (float)(newX * newY) / ( 2*pow(sigma_in, 6)*M_PI );
				elementValue = base * exp((-1)*exponent);
			}

			//yy
			if(typeKernel_in == 3)
			{			
				float base = (pow((float)newY, 2) - pow(sigma_in, 2)) / ( 2*pow(sigma_in, 6)*M_PI );
				elementValue = base * exp((-1)*exponent);
			}

			*( (float*)CV_MAT_ELEM_PTR( *Kernel_out, y, x ) ) = elementValue;
		}
}

void HessianAnalysisEigenvaluesStandard ( IplImage * imageGray_in, float sigma_in, int kernelDimension_in, IplImage * imageGray_out )
{
	CvMat * imageMatrix = cvCreateMat(imageGray_in->height, imageGray_in->width, CV_32FC1 );
	cvConvert( imageGray_in, imageMatrix );
	
	CvMat * kernelXX = cvCreateMat(kernelDimension_in, kernelDimension_in, CV_32FC1);
	calculateKernel(sigma_in, kernelDimension_in, 1, kernelXX);

	CvMat * kernelXY = cvCreateMat(kernelDimension_in, kernelDimension_in, CV_32FC1);
	calculateKernel(sigma_in, kernelDimension_in, 2, kernelXY);

	CvMat * kernelYY = cvCreateMat(kernelDimension_in, kernelDimension_in, CV_32FC1);
	calculateKernel(sigma_in, kernelDimension_in, 3, kernelYY);

	CvMat * Dxx = cvCreateMat(imageGray_in->height, imageGray_in->width, CV_32FC1);
	CvMat * Dxy = cvCreateMat(imageGray_in->height, imageGray_in->width, CV_32FC1);
	CvMat * Dyy = cvCreateMat(imageGray_in->height, imageGray_in->width, CV_32FC1);

	cvFilter2D(imageMatrix, Dxx, kernelXX);
	cvFilter2D(imageMatrix, Dxy, kernelXY);
	cvFilter2D(imageMatrix, Dyy, kernelYY);

	for( int y=0; y < imageGray_in->height; y++ )
	{
		float* vessel = (float*) ( imageGray_out->imageData + y * imageGray_out->widthStep);
			
		for( int x=0; x < imageGray_in->width; x++ )
		{
			double maxEigenvalue = 0;

			float elementDxx = CV_MAT_ELEM( *Dxx, float, y, x );
			float elementDxy = CV_MAT_ELEM( *Dxy, float, y, x );
			float elementDyy = CV_MAT_ELEM( *Dyy, float, y, x );

			//Find Eigenvalues
			double eig1 =  (double)(0.5*( elementDxx + elementDyy + sqrt( (double)( pow(elementDxx - elementDyy, 2) + 4.0*pow(elementDxy,2) )) ));
			double eig2 =  (double)(0.5*( elementDxx + elementDyy - sqrt( (double)( pow(elementDxx - elementDyy, 2) + 4.0*pow(elementDxy,2) )) ));

			//Analyze Eigenvalues' value
			if( abs(eig1) > abs(eig2) )
				maxEigenvalue = eig1;
			else
				maxEigenvalue = eig2;

			if( maxEigenvalue < 0 )
				vessel[x] = 0;
			else
				vessel[x] = (float) maxEigenvalue;
		}
	}

	cvReleaseMat( &imageMatrix);
	cvReleaseMat( &kernelXX);
	cvReleaseMat( &kernelXY);
	cvReleaseMat( &kernelYY);
	cvReleaseMat( &Dxx);
	cvReleaseMat( &Dxy);
	cvReleaseMat( &Dyy);
}

/**
 * Code for thinning a binary image using Zhang-Suen algorithm.
 *
 * Author:  Nash (nash [at] opencv-code [dot] com) 
 * Website: http://opencv-code.com
 */

void thinningIteration(cv::Mat& img, int iter)
{
    CV_Assert(img.channels() == 1);
    CV_Assert(img.depth() != sizeof(uchar));
    CV_Assert(img.rows > 3 && img.cols > 3);

    cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);

    int nRows = img.rows;
    int nCols = img.cols;

    if (img.isContinuous()) {
        nCols *= nRows;
        nRows = 1;
    }

    int x, y;
    uchar *pAbove;
    uchar *pCurr;
    uchar *pBelow;
    uchar *nw, *no, *ne;    // north (pAbove)
    uchar *we, *me, *ea;
    uchar *sw, *so, *se;    // south (pBelow)

    uchar *pDst;

    // initialize row pointers
    pAbove = NULL;
    pCurr  = img.ptr<uchar>(0);
    pBelow = img.ptr<uchar>(1);

    for (y = 1; y < img.rows-1; ++y) {
        // shift the rows up by one
        pAbove = pCurr;
        pCurr  = pBelow;
        pBelow = img.ptr<uchar>(y+1);

        pDst = marker.ptr<uchar>(y);

        // initialize col pointers
        no = &(pAbove[0]);
        ne = &(pAbove[1]);
        me = &(pCurr[0]);
        ea = &(pCurr[1]);
        so = &(pBelow[0]);
        se = &(pBelow[1]);

        for (x = 1; x < img.cols-1; ++x) {
            // shift col pointers left by one (scan left to right)
            nw = no;
            no = ne;
            ne = &(pAbove[x+1]);
            we = me;
            me = ea;
            ea = &(pCurr[x+1]);
            sw = so;
            so = se;
            se = &(pBelow[x+1]);

            int A  = (*no == 0 && *ne == 1) + (*ne == 0 && *ea == 1) + 
                     (*ea == 0 && *se == 1) + (*se == 0 && *so == 1) + 
                     (*so == 0 && *sw == 1) + (*sw == 0 && *we == 1) +
                     (*we == 0 && *nw == 1) + (*nw == 0 && *no == 1);
            int B  = *no + *ne + *ea + *se + *so + *sw + *we + *nw;
            int m1 = iter == 0 ? (*no * *ea * *so) : (*no * *ea * *we);
            int m2 = iter == 0 ? (*ea * *so * *we) : (*no * *so * *we);

            if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                pDst[x] = 1;
        }
    }

    img &= ~marker;
}

void thinning(const cv::Mat& src, cv::Mat& dst)
{
    dst = src.clone();
    dst /= 255;         // convert to binary image

    cv::Mat prev = cv::Mat::zeros(dst.size(), CV_8UC1);
    cv::Mat diff;

    do {
        thinningIteration(dst, 0);
        thinningIteration(dst, 1);
		thinningIteration(dst, 2);
        cv::absdiff(dst, prev, diff);
        dst.copyTo(prev);
    } 
    while (cv::countNonZero(diff) > 0);

    dst *= 255;
}

void read2DCenterlineFromFile( std::vector<Point2d> &centerline_out, int *numPoint_in, std::string Filename_in, int startingPointInstrument_in )
{
//	ifstream read(Filename_in);
//	float x,y,z,i=1,lines;
//	while(read>>x>>y>>z)
//	{
//		Point2d temp;
//		temp.x = x;
//		temp.y = y;
//		centerline_out.push_back(temp);
//		i++;
//	}
//	lines=i-1;

//	*numPoint_in = lines;

//	std::vector<Point2d> centerline_temp;

//	if( startingPointInstrument_in == 0 )
//		reverse(centerline_out.begin(),centerline_out.end());

//	centerline_temp = centerline_out;
//	centerline_out.clear();

//	centerline_out.push_back(centerline_temp[0]);
//	centerline_temp.erase(centerline_temp.begin());

//	for(int i=0; i< *numPoint_in - 1; i++)
//	{
//		Point2d currentPoint;
//		int indexBestPoint;
//		currentPoint = centerline_out[i];
//		float minDistance = FLT_MAX;

//		for(int j=0; j<centerline_temp.size(); j++)
//		{
//			Point2d possiblePoint = centerline_temp[j];
//			float distance = 0;
//			distance = sqrt( pow(possiblePoint.x - currentPoint.x,2) + pow(possiblePoint.y - currentPoint.y,2) );

//			if(distance < minDistance)
//			{
//				minDistance = distance;
//				indexBestPoint = j;
//			}
//		}

//		centerline_out.push_back(centerline_temp[indexBestPoint]);
//		centerline_temp.erase(centerline_temp.begin() + indexBestPoint);
//	}
}

CvScalar random_color(CvRNG* rng)
{
	int color = cvRandInt(rng);
	return CV_RGB(color&255, (color>>8)&255, (color>>16)&255);
}

void fromBlobLabelsToPoints(IplImage *labelImg_in, CvBlobs blobs_in, std::vector<vector<Point2d> > &listBlobs_in)
{
	CvBlobs::iterator it=blobs_in.begin();
	while(it!=blobs_in.end())
	{
		std::vector<Point2d> currentBlob;		

		CvBlob *blob=(*it).second;
		for (unsigned int r=blob->miny-2; r<blob->maxy+2; r++)
			for (unsigned int c=blob->minx-2; c<blob->maxx+2; c++)
			{
				Point2d currentPoint;
				currentPoint.x = c;
				currentPoint.y = r;

				if( ((CvLabel *)(labelImg_in->imageData + (int)r*labelImg_in->widthStep))[(int)c] == blob->label )
					currentBlob.push_back(currentPoint);
			}
		listBlobs_in.push_back(currentBlob);
		it++;
	}
	cvReleaseImage(&labelImg_in);
}

void calculateDistanceSegments(std::vector<Point2d> prevSegment_in, std::vector<Point2d> nextSegment_in, float *distance_out)
{
	*distance_out = 0;
	if(prevSegment_in.size() > nextSegment_in.size())
	{
		for (int i = 0; i < nextSegment_in.size(); i++)
		{
			float minDistance = FLT_MAX;
			Point2d currentPoint = nextSegment_in[i];

			//Search the closest point of groundtruth 2D Model
			for (int j = 0; j < prevSegment_in.size(); j++)
			{
				Point2d temp;
				float distance = 0;

				temp = prevSegment_in[j];
				distance = sqrt( pow(temp.x - currentPoint.x,2) + pow(temp.y - currentPoint.y,2) );

				if(distance < minDistance) 
					minDistance = distance;
			}
			*distance_out = *distance_out + minDistance;
		}
		*distance_out = *distance_out/nextSegment_in.size();
	}
	else
	{
		for (int i = 0; i < prevSegment_in.size(); i++)
		{
			float minDistance = FLT_MAX;
			Point2d currentPoint = prevSegment_in[i];

			//Search the closest point of groundtruth 2D Model
			for (int j = 0; j < nextSegment_in.size(); j++)
			{
				Point2d temp;
				float distance = 0;

				temp = nextSegment_in[j];
				distance = sqrt( pow(temp.x - currentPoint.x,2) + pow(temp.y - currentPoint.y,2) );

				if(distance < minDistance) 
					minDistance = distance;
			}
			*distance_out = *distance_out + minDistance;
		}
		*distance_out = *distance_out/prevSegment_in.size();
	}
}

void findOrientationCloudPointsSorted( std::vector<Point2d> points_in, Point2d *directions_out, float *orientation_out )
{
	int sign1;
	int sign2;

	CvPoint pt1, pt2;
	float line[4];
	int numInterpolatedPoints = points_in.size();

	CvPoint* points = (CvPoint*)malloc( numInterpolatedPoints * sizeof(points[0]));
	CvMat pointMat = cvMat( 1, numInterpolatedPoints, CV_32SC2, points );

	for(int z=0; z < numInterpolatedPoints; z++)
	{
		points[z].x = points_in[z].x;
		points[z].y = points_in[z].y;
	}

	if((points_in[numInterpolatedPoints-1].x - points_in[0].x) >= 0)
		sign1 = 1;
	else
		sign1 = -1;

	if((points_in[numInterpolatedPoints-1].y - points_in[0].y) >= 0)
		sign2 = 1;
	else
		sign2 = -1;

	cvFitLine( &pointMat, CV_DIST_L1, 1, 0.001, 0.001, line );
	line[0] = abs(line[0])*sign1;
	line[1] = abs(line[1])*sign2;

	float x1 = line[0];
	float y1 = line[1];

	orientation_out[0] = ( atan2(y1,x1) * (180 / M_PI) );

	free(points);
	cvReleaseData(&pointMat);
}

void findOrientationCloudPoints( std::vector<Point2d> points_in, Point2d *directions_out, float *orientation_out )
{
	int sign1;
	int sign2;

	CvPoint pt1, pt2;
	float line[4];
	int numInterpolatedPoints = points_in.size();

	CvPoint* points = (CvPoint*)malloc( numInterpolatedPoints * sizeof(points[0]));
	CvMat pointMat = cvMat( 1, numInterpolatedPoints, CV_32SC2, points );

	for(int z=0; z < numInterpolatedPoints; z++)
	{
		points[z].x = points_in[z].x;
		points[z].y = points_in[z].y;
	}

	cvFitLine( &pointMat, CV_DIST_L1, 1, 0.001, 0.001, line );

	float x1 = line[0];
	float y1 = line[1];

	orientation_out[0] = ( atan(y1/x1) * (180 / M_PI) );

	free(points);
	cvReleaseData(&pointMat);
}

void bsplineCubic2D(int numPoints_in, int numCoeffs_in, int numPoints_out, std::vector<double> pointToFitX_in, std::vector<double> pointToFitY_in, std::vector<double> &pointsFittedX_out, std::vector<double> &pointsFittedY_out)
{
//	const size_t n = numPoints_in;
//	const size_t ncoeffs = numCoeffs_in;
//	const size_t nbreak = ncoeffs - 2;

//	size_t i, j;
//	gsl_bspline_workspace *bw1;
//	gsl_bspline_workspace *bw2;
//	gsl_vector *B1;
//	gsl_vector *B2;

//	gsl_vector *c1, *w1;
//	gsl_vector *c2, *w2;
//	gsl_vector *x1, *y1;
//	gsl_vector *x2, *y2;
//	gsl_matrix *X1, *cov1;
//	gsl_matrix *X2, *cov2;

//	gsl_multifit_linear_workspace *mw1;
//	gsl_multifit_linear_workspace *mw2;

//	double chisq1, chisq2, Rsq, dof, tss;
     
//	gsl_rng_env_setup();
     
//	/* allocate a cubic bspline workspace (k = 4) */
//	bw1 = gsl_bspline_alloc(4, nbreak);
//	bw2 = gsl_bspline_alloc(4, nbreak);
       
//	B1 = gsl_vector_alloc(ncoeffs);
//	B2 = gsl_vector_alloc(ncoeffs);
       
//	x1 = gsl_vector_alloc(n);
//	y1 = gsl_vector_alloc(n);
//	x2 = gsl_vector_alloc(n);
//	y2 = gsl_vector_alloc(n);
      
//	X1 = gsl_matrix_alloc(n, ncoeffs);
//	X2 = gsl_matrix_alloc(n, ncoeffs);
       
//	c1 = gsl_vector_alloc(ncoeffs);
//	w1 = gsl_vector_alloc(n);
//	c2 = gsl_vector_alloc(ncoeffs);
//	w2 = gsl_vector_alloc(n);
       
//	cov1 = gsl_matrix_alloc(ncoeffs, ncoeffs);
//	cov2 = gsl_matrix_alloc(ncoeffs, ncoeffs);
       
//	mw1 = gsl_multifit_linear_alloc(n, ncoeffs);
//	mw2 = gsl_multifit_linear_alloc(n, ncoeffs);
    
//	/* this is the data to be fitted */
//	for (i = 0; i < n; ++i)
//	{

//		double xi = pointToFitX_in[i];
//		double yi = pointToFitY_in[i];
         
//		gsl_vector_set(x1, i, i);
//		gsl_vector_set(y1, i, xi);
//		gsl_vector_set(x2, i, i);
//		gsl_vector_set(y2, i, yi);

//		gsl_vector_set(w1, i, 1.0 );
//		gsl_vector_set(w2, i, 1.0 );

//	}
     
//	/* use uniform breakpoints on [0, 15] */
//	gsl_bspline_knots_uniform(0.0, n*1.0, bw1);
//	gsl_bspline_knots_uniform(0.0, n*1.0, bw2);

//	/* construct the fit matrix X */
//	for (i = 0; i < n; ++i)
//	{
//		double xi1 = gsl_vector_get(x1, i);
//		double xi2 = gsl_vector_get(x2, i);
     
//		/* compute B_j(xi) for all j */
//		gsl_bspline_eval(xi1, B1, bw1);
//		gsl_bspline_eval(xi2, B2, bw2);
     
//		/* fill in row i of X */
//		for (j = 0; j < ncoeffs; ++j)
//		{
	
//			double Bj1 = gsl_vector_get(B1, j);
//			double Bj2 = gsl_vector_get(B2, j);
               
//			gsl_matrix_set(X1, i, j, Bj1);
//			gsl_matrix_set(X2, i, j, Bj2);
//		}
//	}
     
//	/* do the fit */
//	gsl_multifit_wlinear(X1, w1, y1, c1, cov1, &chisq1, mw1);
//	gsl_multifit_wlinear(X2, w2, y2, c2, cov2, &chisq2, mw2);

//	/* Save the smoothed curve */
//	double t_step, pointFit1, pointFit2, yerr1, yerr2, t_stepDim;

//	t_stepDim = (n*1.0) / numPoints_out;
     
//	for (t_step = 0.0; t_step < n; t_step += t_stepDim)
//	{
//		gsl_bspline_eval(t_step, B1, bw1);
//		gsl_bspline_eval(t_step, B2, bw2);

//		gsl_multifit_linear_est(B1, c1, cov1, &pointFit1, &yerr1);
//		gsl_multifit_linear_est(B2, c2, cov2, &pointFit2, &yerr1);

//		pointsFittedX_out.push_back(pointFit1);
//		pointsFittedY_out.push_back(pointFit2);
//	}

//	gsl_bspline_free(bw1);
//	gsl_vector_free(B1);
//	gsl_vector_free(x1);
//	gsl_vector_free(y1);
//	gsl_matrix_free(X1);
//	gsl_vector_free(c1);
//	gsl_vector_free(w1);
//	gsl_matrix_free(cov1);
//	gsl_multifit_linear_free(mw1);

//	gsl_bspline_free(bw2);
//	gsl_vector_free(B2);
//	gsl_vector_free(x2);
//	gsl_vector_free(y2);
//	gsl_matrix_free(X2);
//	gsl_vector_free(c2);
//	gsl_vector_free(w2);
//	gsl_matrix_free(cov2);
//	gsl_multifit_linear_free(mw2);
}

void extractStraightSegmentsFromAConnectedRegion( int maxNumPixelPerSegment_in, double slopeThreshold_in, IplImage *image_in, std::vector<Point2d> blob_in, int *currentIndexSegment_in, int minSizeSegment_in, std::vector<straightSegment> &currentListSegments_out )
{
	if(blob_in.size() == 0)
		return;

	IplImage *exploringMask;

	std::vector<Point3d> seeds;
	std::vector<Point3d> remainingSeeds;

	Point3d seed;

	//Set the seeds for the region growing
	if( *currentIndexSegment_in != (-1))
	{
		seed.x = blob_in[0].x;
		seed.y = blob_in[0].y;
		seed.z = 0;
	}
	else
	{
		seed.x = blob_in[blob_in.size()-1].x;
		seed.y = blob_in[blob_in.size()-1].y;
		seed.z = 0;
	}

	seeds.push_back(seed);

	exploringMask = cvCreateImage( cvGetSize(image_in), IPL_DEPTH_8U, 1 );
	cvSet(exploringMask, cvScalar(0));

	bool exploring = true;
	while(exploring)
	{
		std::vector<Point3d> resultRegionGrowing;
		std::vector<Point3d> lastNeighborhood;
		straightSegment currentStraightSegment;

		//Region growing considering neibouring pixels with the same label and belonging to a straigth configuration
		regionGrowing ( maxNumPixelPerSegment_in, slopeThreshold_in, image_in, exploringMask, seeds, resultRegionGrowing, lastNeighborhood );

		if(resultRegionGrowing.size() > minSizeSegment_in)
		{
			(*currentIndexSegment_in)++;
			currentStraightSegment.id = *currentIndexSegment_in;

			for(int i=0; i < resultRegionGrowing.size(); i++ )
			{
				Point2d currentPoint;

				currentPoint.x = resultRegionGrowing[i].x;
				currentPoint.y = resultRegionGrowing[i].y;
				currentStraightSegment.points.push_back(currentPoint);
			}

			currentListSegments_out.push_back(currentStraightSegment);
		}

		for(int i=0; i < lastNeighborhood.size(); i++ )
			remainingSeeds.push_back(lastNeighborhood[i]);

		if( remainingSeeds.size() == 0 )
		{
			exploring = false;
			break;
		}
		else
		{
			seeds.clear();
			seeds.push_back(remainingSeeds[0]);
			remainingSeeds.erase(remainingSeeds.begin() + 0);
		}
	}
	
    //cvReleaseImage(&exploringMask);
}

void extractStraightSegmentsFromAConnectedRegionGT( int maxNumPixelPerSegment_in, double slopeThreshold_in, std::vector<Point2d> blob_in, int *currentIndexSegment_in, int minSizeSegment_in, std::vector<straightSegment> &currentListSegments_out )
{
	int minDistancePointsToCalculateSlope = 4;
	reverse(blob_in.begin(), blob_in.end());
	
	while( blob_in.size() > 0 )
	{
		straightSegment currentSegment;
		
		currentSegment.points.push_back(blob_in[0]);
		blob_in.erase(blob_in.begin() + 0);

		double slope = 0.0;
		double slopeDiff = 0.0;
		while( (blob_in.size() > 0) && (currentSegment.points.size() < maxNumPixelPerSegment_in) && (slopeDiff <= slopeThreshold_in) )
		{		
			Point2d refVector; refVector.x = 1; refVector.y = 0;
			Point2d currentVector;
			Point2d currentPoint = blob_in[0];
			
			if(currentSegment.points.size() < minDistancePointsToCalculateSlope)
			{
				currentVector.x = currentPoint.x - currentSegment.points[0].x; 
				currentVector.y = currentPoint.y - currentSegment.points[0].y;
			}
			else
			{
				currentVector.x = currentPoint.x - currentSegment.points[currentSegment.points.size()-minDistancePointsToCalculateSlope].x; 
				currentVector.y = currentPoint.y - currentSegment.points[currentSegment.points.size()-minDistancePointsToCalculateSlope].y;
			}

			double currentSlope = 180/M_PI*atan2( (refVector.x*currentVector.y - refVector.y*currentVector.x) , (refVector.x*currentVector.x + refVector.y*currentVector.y) );

			if(currentSegment.points.size() < minDistancePointsToCalculateSlope)
				slope = currentSlope;

			slopeDiff = abs(currentSlope - slope);

			if( (currentSegment.points.size() < maxNumPixelPerSegment_in) && (slopeDiff <= slopeThreshold_in) )
			{
				currentSegment.points.push_back(currentPoint);
				blob_in.erase(blob_in.begin() + 0);
			}
		}

		if(currentSegment.points.size() > minSizeSegment_in)
		{
			(*currentIndexSegment_in)++;
			currentSegment.id = *currentIndexSegment_in;
			currentListSegments_out.push_back(currentSegment);
		}
	}
}

void refineSegmentsTip ( int minSizeSegment_in, int numPixelTip_in, std::vector<straightSegment> &listSegments_inout )
{
	int indexLastSegment		= listSegments_inout.size() - 1;
	int indexSecondLastSegment	= listSegments_inout.size() - 2;
	int indexThirdLastSegment	= listSegments_inout.size() - 3;
	int indexFourthLastSegment	= listSegments_inout.size() - 4;

	straightSegment tip, lastSegment, secondLastSegment, thirdLastSegment, fourthLastSegment ;

	lastSegment	= listSegments_inout[indexLastSegment];
	 
	if(listSegments_inout.size() >= 2)
		secondLastSegment = listSegments_inout[indexSecondLastSegment];

	if(listSegments_inout.size() >= 3)
		thirdLastSegment = listSegments_inout[indexThirdLastSegment];
		
	if(listSegments_inout.size() >= 4)
		fourthLastSegment = listSegments_inout[indexFourthLastSegment];

	//Refine the Segment in order to have the last straight segment with fixed length = TIP
	if( lastSegment.points.size() == numPixelTip_in )
		;
	//Longer Tip Segment
	else if( lastSegment.points.size() > numPixelTip_in )
	{
		for(int i=0; i < numPixelTip_in; i++)
		{
			int indexLastPoint = lastSegment.points.size() - 1;
			tip.points.push_back( lastSegment.points[indexLastPoint] );
			lastSegment.points.erase(lastSegment.points.begin() + indexLastPoint);
		}

		if(lastSegment.points.size() < minSizeSegment_in)
		{
			for(int i=0; i < lastSegment.points.size(); i++)
				listSegments_inout[indexSecondLastSegment].points.push_back(lastSegment.points[i]);
			
			tip.id = lastSegment.id;
			reverse(tip.points.begin(), tip.points.end());
			listSegments_inout[indexLastSegment] = tip;
		}
		else
		{
			listSegments_inout[indexLastSegment] = lastSegment;
			tip.id = lastSegment.id + 1;
			reverse(tip.points.begin(), tip.points.end());
			listSegments_inout.push_back(tip);
		}
	}
	//Shorter Tip Segment
	else
	{
		tip = lastSegment;
		
		int neededPixels = numPixelTip_in - tip.points.size();	
		reverse(tip.points.begin(), tip.points.end());		
		listSegments_inout.erase(listSegments_inout.begin() + indexLastSegment);
		
		while((neededPixels != 0) && (secondLastSegment.points.size() > 0))
		{
			int indexLastPoint = secondLastSegment.points.size() - 1;
			tip.points.push_back( secondLastSegment.points[indexLastPoint] );
			secondLastSegment.points.erase(secondLastSegment.points.begin() + indexLastPoint);			
			neededPixels--;
		}
		
		if(neededPixels == 0)
		{
			if(secondLastSegment.points.size() < minSizeSegment_in)
			{
				for(int i=0; i < secondLastSegment.points.size(); i++)
					listSegments_inout[indexThirdLastSegment].points.push_back(secondLastSegment.points[i]);
				
				tip.id = lastSegment.id - 1;
				reverse(tip.points.begin(), tip.points.end());
				listSegments_inout[indexSecondLastSegment] = tip;
			}
			else
			{
				listSegments_inout[indexSecondLastSegment] = secondLastSegment;
				reverse(tip.points.begin(), tip.points.end());
				listSegments_inout.push_back(tip);
			}
		}
		else
		{
			listSegments_inout.erase(listSegments_inout.begin() + indexSecondLastSegment);
			
			while((neededPixels != 0) && (thirdLastSegment.points.size() > 0))
			{
				int indexLastPoint = thirdLastSegment.points.size() - 1;
				tip.points.push_back( thirdLastSegment.points[indexLastPoint] );
				thirdLastSegment.points.erase(thirdLastSegment.points.begin() + indexLastPoint);			
				neededPixels--;
			}
			
			if(neededPixels == 0)
			{
				if(thirdLastSegment.points.size() < minSizeSegment_in)
				{
					for(int i=0; i < thirdLastSegment.points.size(); i++)
						listSegments_inout[indexFourthLastSegment].points.push_back(thirdLastSegment.points[i]);
					
					tip.id = lastSegment.id - 2;
					reverse(tip.points.begin(), tip.points.end());
					listSegments_inout[indexThirdLastSegment] = tip;
				}
				else
				{
					listSegments_inout[indexThirdLastSegment] = thirdLastSegment;
					tip.id = lastSegment.id - 1;
					reverse(tip.points.begin(), tip.points.end());
					listSegments_inout.push_back(tip);
				}
			}
			else
			{
				listSegments_inout.erase(listSegments_inout.begin() + indexThirdLastSegment);
				
				while((neededPixels != 0) && (fourthLastSegment.points.size() > 0))
				{
					int indexLastPoint = fourthLastSegment.points.size() - 1;
					tip.points.push_back( fourthLastSegment.points[indexLastPoint] );
					fourthLastSegment.points.erase(fourthLastSegment.points.begin() + indexLastPoint);			
					neededPixels--;
				}
				
				if(neededPixels == 0)
				{
					if(fourthLastSegment.points.size() < minSizeSegment_in)
					{
						for(int i=0; i < fourthLastSegment.points.size(); i++)
							listSegments_inout[indexFourthLastSegment - 1].points.push_back(fourthLastSegment.points[i]);
						
						tip.id = lastSegment.id - 3;
						reverse(tip.points.begin(), tip.points.end());
						listSegments_inout[indexFourthLastSegment] = tip;
					}
					else
					{
						listSegments_inout[indexFourthLastSegment] = fourthLastSegment;
						tip.id = lastSegment.id - 2;
						reverse(tip.points.begin(), tip.points.end());
						listSegments_inout.push_back(tip);
					}
				}
			}
		}
	}
}

void regionGrowing ( int maxNumPixelPerSegment_in, double slopeThreshold_in, IplImage * image_in, IplImage * exploringMask_in, std::vector<Point3d> seeds_in, std::vector<Point3d> &resultRegionGrowing_out, std::vector<Point3d> &lastNeighborhood_out )
{	
	int minDistancePointsToCalculateSlope = 4;
	double slope = 0.0;
	
	std::vector<Point3d> stackGrowing;

	for(int i=0; i<seeds_in.size(); i++)
	{
		int X,Y;

		X = seeds_in[i].x;
		Y = seeds_in[i].y;

		Point3d temp;

		temp.x = seeds_in[i].x;
		temp.y = seeds_in[i].y;
		temp.z = 0;
		
		resultRegionGrowing_out.push_back(temp);

		stackGrowing.push_back(temp);
		((uchar *)(exploringMask_in->imageData + (int)temp.y*exploringMask_in->widthStep))[(int)temp.x] = 1;
	}

	bool  running = true;
	while( (!stackGrowing.empty()) && (running == true) )
	{
		Point3d currentPoint;

		currentPoint = stackGrowing[stackGrowing.size()-1];
		stackGrowing.erase(stackGrowing.begin() + stackGrowing.size() - 1);	

		std::vector<Point3d> neighborhood;

		//Find Unexplored Neighborhood
		for(int dx=-1; dx < 2; dx++)
			for(int dy=-1; dy < 2; dy++)
			{
				if((dx == 0) && (dy == 0))
					;
				else
				{
					Point3d newPoint;
					int X,Y;

					newPoint.x = dx + currentPoint.x;
					newPoint.y = dy + currentPoint.y;
					newPoint.z = currentPoint.z + 1;
			
					float currentMask = ((uchar *)(exploringMask_in->imageData + (int)newPoint.y*exploringMask_in->widthStep))[(int)newPoint.x];

					if(currentMask != 1)
					{
						X = newPoint.x;
						Y = newPoint.y;

						double pixelValue = ((uchar *)(image_in->imageData + Y*image_in->widthStep))[X];

						if( pixelValue > 0 ) 
							neighborhood.push_back(newPoint);
					}
				}
			}
		//There are not neibour pixels
		if( neighborhood.size() == 0 )
			running = false;

		//Analyze the neighborhood to decide next step in the growing 
		else if( neighborhood.size() == 2 || neighborhood.size() == 1 ) 
		{

			double diffBetweenNeighborhoood = abs(neighborhood[0].x - neighborhood[neighborhood.size()-1].x) + abs(neighborhood[0].y - neighborhood[neighborhood.size()-1].y);

			if( diffBetweenNeighborhoood > 1 )
			{
				for(int z=0; z<neighborhood.size(); z++)
					lastNeighborhood_out.push_back(neighborhood[z]);
				running = false;
			}
			else
			{
				if( (neighborhood.size() == 2) && ((abs(neighborhood[0].x - currentPoint.x) == 1) && (abs(neighborhood[0].y - currentPoint.y) == 1)) )
					std::reverse(neighborhood.begin(),neighborhood.end());			

				((uchar *)(exploringMask_in->imageData + (int)neighborhood[0].y*exploringMask_in->widthStep))[(int)neighborhood[0].x] = 1;

				//Calculate Slope new Vector
				Point2d refVector; refVector.x = 1; refVector.y = 0;
				Point2d currentVector;
			
				if(resultRegionGrowing_out.size() < minDistancePointsToCalculateSlope)
				{
					currentVector.x = neighborhood[0].x - seeds_in[0].x; 
					currentVector.y = neighborhood[0].y - seeds_in[0].y;
				}
				else
				{
					currentVector.x = neighborhood[0].x - resultRegionGrowing_out[resultRegionGrowing_out.size()-minDistancePointsToCalculateSlope].x; 
					currentVector.y = neighborhood[0].y - resultRegionGrowing_out[resultRegionGrowing_out.size()-minDistancePointsToCalculateSlope].y;
				}

				double currentSlope = 180/M_PI*atan2( (refVector.x*currentVector.y - refVector.y*currentVector.x) , (refVector.x*currentVector.x + refVector.y*currentVector.y) );

				if(resultRegionGrowing_out.size() < minDistancePointsToCalculateSlope)
					slope = currentSlope;

				double slopeDiff = abs(currentSlope - slope);

				if(resultRegionGrowing_out.size() > maxNumPixelPerSegment_in)
				{
					lastNeighborhood_out.push_back(neighborhood[0]); 
					running = false;
				}
				else if( slopeDiff <= slopeThreshold_in )
				{
					stackGrowing.push_back(neighborhood[0]);
					resultRegionGrowing_out.push_back(neighborhood[0]);
				}
				else
				{
					lastNeighborhood_out.push_back(neighborhood[0]); 
					running = false;
				}
			}
		}
		else if( neighborhood.size() >= 3 )
		{
			for(int z=0; z<neighborhood.size(); z++)
				lastNeighborhood_out.push_back(neighborhood[z]);
			running = false;
		}
	}
}

void extractBlobs( int minSizeSegment_in, int minDistCentrelineGT_in, IplImage *image_in, std::vector<Point2d> centrelineGT_in, std::vector<vector<Point2d> > &listBlobs_out, std::vector<straightSegment> &listSegmentsFilteredOut_out )
{
	int maxAreaAcceptableForBlob		= 100000;  
	int maxAreaAcceptableForBlobNoDist	= 300;		//300 

    std::vector<vector<Point2d> > listBlobs;
	
	CvBlobs blobs;
	CvBlobs blobsFilteredOut;
	
	IplImage *labelImg=cvCreateImage(cvGetSize(image_in), IPL_DEPTH_LABEL, 1);
	int result=cvLabel(image_in, labelImg, blobs);
			
	CvBlobs::iterator it=blobs.begin();
	while(it!=blobs.end())
	{
		CvBlob *blob=(*it).second;
		CvPoint2D64f position = blob->centroid;
		double area = blob->area;
				
		if(area > minSizeSegment_in && area < maxAreaAcceptableForBlob)
		{
			float minDistance = FLT_MAX;
			for( int j=0; j < centrelineGT_in.size(); j++ )
			{
				float distCenterline = sqrt(pow(centrelineGT_in[j].x - position.x,2) + pow(centrelineGT_in[j].y - position.y,2));
				if(distCenterline < minDistance)
					minDistance = distCenterline;
			}
			if((minDistance < minDistCentrelineGT_in) || (maxAreaAcceptableForBlobNoDist < area))
				++it;
			else
			{
				straightSegment currentBlobErased;				
				for (unsigned int r=blob->miny-2; r<blob->maxy+2; r++)
					for (unsigned int c=blob->minx-2; c<blob->maxx+2; c++)
					{
						Point2d currentPoint;
						currentPoint.x = c;
						currentPoint.y = r;

						if( ((CvLabel *)(labelImg->imageData + (int)r*labelImg->widthStep))[(int)c] == blob->label )
							currentBlobErased.points.push_back(currentPoint);

					}
				
				listSegmentsFilteredOut_out.push_back(currentBlobErased);

				cvReleaseBlob(blob);
				CvBlobs::iterator tmp=it;
				++it;
				blobs.erase(tmp);
			}			
		}	
		else
		{
			cvReleaseBlob(blob);
			CvBlobs::iterator tmp=it;
			++it;
			blobs.erase(tmp);
		}
	}
	fromBlobLabelsToPoints(labelImg, blobs, listBlobs_out);
}

/*
void createSmoothCurveFromPoints( int distanceNodes_in, std::vector<Point2d> &pointsHp_inout, int indexCurve_in, IplImage *currentImage_in, int startingPointInstrument_in, std::vector<int> &indexCurve_out )
{								
	int numberPoints_out = 10000;
	int numberFitCoeff_out = 30;	if(pointsHp_inout.size() < numberFitCoeff_out)	numberFitCoeff_out = pointsHp_inout.size();	
	int numberPointGraph = pointsHp_inout.size();
	int numberPoints_in = pointsHp_inout.size();

	int minY = INT_MAX;
	int maxY = INT_MIN;
	for (int i = 0; i < pointsHp_inout.size(); i++)
	{
		Point2d currentPoint = pointsHp_inout[i];
		int x_point, y_point;		
		x_point = (int)currentPoint.x;
		y_point = (int)currentPoint.y;
		if(y_point < minY)
			minY = y_point;
		if(y_point > maxY)
			maxY = y_point;
	}

	Point2d lastPoint = pointsHp_inout[pointsHp_inout.size()-1];

	std::vector<double> pointToFitX_in;
	std::vector<double> pointToFitY_in;
	std::vector<double> pointsFittedX_out;
	std::vector<double> pointsFittedY_out;

	for (int i = 0; i < numberPoints_in; ++i)
	{
		double xi = pointsHp_inout[i].x;
		double yi = pointsHp_inout[i].y;
		pointToFitX_in.push_back(xi);
		pointToFitY_in.push_back(yi);
	}
	pointsHp_inout.clear();

	bsplineCubic2D(numberPoints_in, numberFitCoeff_out, numberPoints_out, pointToFitX_in, pointToFitY_in, pointsFittedX_out, pointsFittedY_out);

	double currentDistance = 0;
	int index = 0;
	int numberOfGoodPoints = 0;

	IplImage *mask;
	mask = cvCreateImage( cvGetSize(currentImage_in), IPL_DEPTH_8U, 0 );
	cvSet(mask, cvScalar(0));

	//Discretize the curve in the Image space + Check possible overfitting of the spline
	for (int i = 0; i < numberPoints_out; ++i)
	{
		Point2d currentPoint;
		currentPoint.x = pointsFittedX_out[i];
		currentPoint.y = pointsFittedY_out[i];

		int x_point, y_point;
		x_point = (int)currentPoint.x;
		y_point = (int)currentPoint.y;

		if( startingPointInstrument_in == 0 )
		{
			if( maxY < y_point )
				break;
		}
		else if( startingPointInstrument_in == 1 )
		{
			if( minY > y_point )
				break;
		}

		int intensityValue = ((uchar *)(mask->imageData + y_point*mask->widthStep))[x_point];
		if( intensityValue != 255 )
		{
			Point2d currentGoodPoint; currentGoodPoint.x = x_point; currentGoodPoint.y = y_point;
			pointsHp_inout.push_back(currentGoodPoint);
			numberOfGoodPoints++;
			((uchar *)(mask->imageData + y_point*mask->widthStep))[x_point] = 255;
		}
	}
	numberPoints_out = numberOfGoodPoints;

	indexCurve_out.push_back(0);
	index++;
	for (int i = 1; i < numberPoints_out; i++ )
	{
		Point2d currentPoint, prevPoint;
		prevPoint.x = pointsHp_inout[i-1].x;
		prevPoint.y = pointsHp_inout[i-1].y;
		currentPoint.x = pointsHp_inout[i].x;
		currentPoint.y = pointsHp_inout[i].y;

		double distance = sqrt( pow(prevPoint.x - currentPoint.x,2) + pow(prevPoint.y - currentPoint.y,2) );
		currentDistance = currentDistance + distance;

		if( currentDistance >= distanceNodes_in )
		{
			currentDistance = 0;
			index++;
			indexCurve_out.push_back(i);

			if( index == numberPointGraph-1)
			{
				indexCurve_out.push_back(numberPointGraph-1);
				break;
			}
		}
	}
	cvReleaseImage(&mask);
}*/

void createSmoothCurveFromPoints2( int distanceNodes_in, std::vector<Point2d> &pointsHp_inout, int indexCurve_in, IplImage *currentImage_in, int startingPointInstrument_in, std::vector<int> &indexCurve_out )
{			
	int numberPointGraph = pointsHp_inout.size();
	std::vector<Point2d> tmpInterpolation;
	std::vector<Point2d> lol; lol = pointsHp_inout;

	for (int i = 0; i < pointsHp_inout.size(); i++)
	{
		Point2d currentPoint = pointsHp_inout[i];
				
		if(i > 0)
		{
			Point2d previousPoint = pointsHp_inout[i-1];
			
			double distance = sqrt( pow(currentPoint.x - previousPoint.x,2) + pow(currentPoint.y - previousPoint.y,2) );
			
			//Linear interpolation
			if(	distance >= 1 )
			{
				Point2d vector;
	
				vector = currentPoint - previousPoint;
				double module = sqrt(pow(vector.x,2) + pow(vector.y,2));

				vector.x = vector.x / module; vector.y = vector.y / module;

				double minDistance = 0.1;
				double step = 0.1;
				
				double indexStep = 1.0;
				while( true )
				{
					Point2d newPoint = (step*indexStep*vector) + previousPoint;
					tmpInterpolation.push_back(newPoint);
					indexStep++;
					
					double newDistance = sqrt( pow(currentPoint.x - newPoint.x,2) + pow(currentPoint.y - newPoint.y,2) );
					if( newDistance < minDistance )
						break;
				}

			}

		tmpInterpolation.push_back(currentPoint);
		}
	}
	
	pointsHp_inout.clear();
	
	IplImage *mask;
	mask = cvCreateImage( cvGetSize(currentImage_in), IPL_DEPTH_8U, 0 );
	cvSet(mask, cvScalar(0));
	
	int numberOfGoodPoints = 0;

	//Discretize the curve in the Image space
	for (int i = 0; i < tmpInterpolation.size(); ++i)
	{
		Point2d currentPoint;
		currentPoint.x = tmpInterpolation[i].x;
		currentPoint.y = tmpInterpolation[i].y;

		int x_point, y_point;
		x_point = (int)currentPoint.x;
		y_point = (int)currentPoint.y;

		int intensityValue = ((uchar *)(mask->imageData + y_point*mask->widthStep))[x_point];
		if( intensityValue != 255 )
		{
			Point2d currentGoodPoint; currentGoodPoint.x = x_point; currentGoodPoint.y = y_point;

			pointsHp_inout.push_back(currentGoodPoint);
			numberOfGoodPoints++;
			((uchar *)(mask->imageData + y_point*mask->widthStep))[x_point] = 255;
		}
	}

	int index = 0;
	double currentDistance = 0;
	indexCurve_out.push_back(0);
	index++;

	for (int i = 1; i < numberOfGoodPoints; i++ )
	{
		Point2d currentPoint, prevPoint;
		prevPoint.x = pointsHp_inout[i-1].x;
		prevPoint.y = pointsHp_inout[i-1].y;
		currentPoint.x = pointsHp_inout[i].x;
		currentPoint.y = pointsHp_inout[i].y;

		double distance = sqrt( pow(prevPoint.x - currentPoint.x,2) + pow(prevPoint.y - currentPoint.y,2) );
		currentDistance = currentDistance + distance;

		if( currentDistance >= distanceNodes_in )
		{
			currentDistance = 0;
			index++;
			indexCurve_out.push_back(i);

			if( index == numberPointGraph-1)
			{
				indexCurve_out.push_back(numberPointGraph-1);
				break;
			}
		}
	}
	cvReleaseImage(&mask);
}

void printLinenessMap( IplImage *currentImage_in, std::string fileName_in, int indexFrame_in )
{
	float maxPixelValue = FLT_MIN;
	float minPixelValue = FLT_MAX;

	IplImage* image_printing = cvCreateImage( cvGetSize(currentImage_in), IPL_DEPTH_8U, 1);

	for (int i = 0; i < currentImage_in->height; i++ )
		for (int j = 0; j < currentImage_in->width; j++ )
		{
			float pixelValue; 
			pixelValue = ((float *)(currentImage_in->imageData + i*currentImage_in->widthStep))[j];
			if( pixelValue < minPixelValue)
				minPixelValue = pixelValue;
			if( pixelValue > maxPixelValue)
				maxPixelValue = pixelValue;
		}

	float step = (255.0/(maxPixelValue - minPixelValue));

	for (int i = 0; i < image_printing->height; i++ )
		for (int j = 0; j < image_printing->width; j++ )
		{
			float valueCurrentPixel = ((float *)(currentImage_in->imageData + i*currentImage_in->widthStep))[j];
			((uchar *)(image_printing->imageData + i*image_printing->widthStep))[j] = (int)(step * 1.0 * (valueCurrentPixel - minPixelValue));
		}

	std::stringstream ss;
	std::string imageName;

	ss << fileName_in << "HessianMap/currentLinenessMap" << indexFrame_in << ".jpg";
	imageName = ss.str();
	const char * imageNameChar = imageName.c_str();

	cvSaveImage(imageNameChar, image_printing);
	cvReleaseImage(&image_printing);
}

void printBinaryImage( IplImage *currentImage_in, std::string fileName_in, int indexFrame_in )
{
	std::stringstream ss;
	std::string imageName;

	ss << fileName_in << "ThinnedImages/thinned" << indexFrame_in << ".jpg";
	imageName = ss.str();
	const char * imageNameChar = imageName.c_str();

	cvSaveImage(imageNameChar, currentImage_in);
}

void printSegments( IplImage *currentImage_in, int type_in, std::string fileName_in, int indexFrame_in, std::vector<straightSegment> listSegments_in )
{
	IplImage *image_printing;

	image_printing = cvCreateImage( cvGetSize(currentImage_in), IPL_DEPTH_8U, 3 );
	cvSet(image_printing, cvScalar(255, 255, 255));

	ofstream myfile;

	if( type_in == 4 )
	{
		std::stringstream ss0;
		std::string imageName0;

		ss0 << fileName_in << "SegmentsPoints/SegPoints_Frame" << indexFrame_in << ".txt";
		imageName0 = ss0.str();
		const char * imageNameChar0 = imageName0.c_str();
		
		myfile.open(imageNameChar0);
	}

	CvRNG rng;
	for (unsigned int r=0; r<listSegments_in.size(); r++)
	{
		CvScalar color = random_color(&rng);
		for (unsigned int c=0; c<listSegments_in[r].points.size(); c++)
		{	
			if( type_in == 4 )
				myfile << listSegments_in[r].points[c].x << " " << listSegments_in[r].points[c].y << " " << 255 << "\n";

			if( c != 0 && c!= listSegments_in[r].points.size()-1 )
			{
				if( (color.val[2] > 250) && (color.val[1] > 250) && (color.val[0] > 250) )
				{
					color.val[2] = 0; 
					color.val[1] = 0;
					color.val[0] = 0;
				}

				((uchar *)(image_printing->imageData + (int)listSegments_in[r].points[c].y*image_printing->widthStep))[(int)listSegments_in[r].points[c].x*image_printing->nChannels + 0]= color.val[2]; // B
				((uchar *)(image_printing->imageData + (int)listSegments_in[r].points[c].y*image_printing->widthStep))[(int)listSegments_in[r].points[c].x*image_printing->nChannels + 1]= color.val[1]; // G
				((uchar *)(image_printing->imageData + (int)listSegments_in[r].points[c].y*image_printing->widthStep))[(int)listSegments_in[r].points[c].x*image_printing->nChannels + 2]= color.val[0]; // R
			}
			else
			{
				((uchar *)(image_printing->imageData + (int)listSegments_in[r].points[c].y*image_printing->widthStep))[(int)listSegments_in[r].points[c].x*image_printing->nChannels + 0]= 0; // B
				((uchar *)(image_printing->imageData + (int)listSegments_in[r].points[c].y*image_printing->widthStep))[(int)listSegments_in[r].points[c].x*image_printing->nChannels + 1]= 0; // G
				((uchar *)(image_printing->imageData + (int)listSegments_in[r].points[c].y*image_printing->widthStep))[(int)listSegments_in[r].points[c].x*image_printing->nChannels + 2]= 255; // R
			}
		}
	}

	if( type_in == 4 )
		myfile.close();

	std::stringstream ss;
	std::string imageName;

	if( type_in == 0 )
		ss << fileName_in << "ListSegments/listSegmentsGT" << indexFrame_in << ".jpg";
	else if( type_in == 1 )
		ss << fileName_in << "ListSegments/listSegments" << indexFrame_in << ".jpg";
	else if( type_in == 2 )
		ss << fileName_in << "ListSegments/listSegmentsFilteredOutTooFar" << indexFrame_in << ".jpg";
	else if( type_in == 3 )
		ss << fileName_in << "ListSegments/listSegmentsFilteredOut" << indexFrame_in << ".jpg";
	else if( type_in == 4 )
		ss << fileName_in << "ListSegments/SEGlets" << indexFrame_in << ".jpg";
	
	imageName = ss.str();
	const char * imageNameChar = imageName.c_str();

	cvSaveImage(imageNameChar, image_printing);
	cvReleaseImage(&image_printing);
}

void printBackgroundMap( IplImage *currentImage_in, std::string fileName_in, int indexFrame_in )
{
	float thresholdBackgroundMap = 0;
	float maxPixelValue = FLT_MIN;
	float minPixelValue = FLT_MAX;

	IplImage* image_printing = cvCreateImage( cvGetSize(currentImage_in), IPL_DEPTH_8U, 1);

	for (int i = 0; i < currentImage_in->height; i++ )
		for (int j = 0; j < currentImage_in->width; j++ )
		{
			float pixelValue; 
			pixelValue = ((float *)(currentImage_in->imageData + i*currentImage_in->widthStep))[j];
			if( pixelValue < minPixelValue)
				minPixelValue = pixelValue;
			if( pixelValue > maxPixelValue)
				maxPixelValue = pixelValue;
		}

	float step = (255.0/(maxPixelValue - minPixelValue));

	for (int i = 0; i < image_printing->height; i++ )
		for (int j = 0; j < image_printing->width; j++ )
		{
			float valueCurrentPixel = ((float *)(currentImage_in->imageData + i*currentImage_in->widthStep))[j];
			((uchar *)(image_printing->imageData + i*image_printing->widthStep))[j] = (int)(step * 1.0 * (valueCurrentPixel - minPixelValue));
		}
	
	std::stringstream ss;
	std::string imageName;

	ss << fileName_in << "BackgroundMap/backgroundMap" << indexFrame_in << ".jpg";

	imageName = ss.str();
	const char * imageNameChar = imageName.c_str();

	cvSaveImage(imageNameChar, image_printing);

	cvReleaseImage(&image_printing);
}

void printSegmentsHps( IplImage *currentImage_in, std::string fileName_in, int indexFrame_in, std::vector<straightSegment> listSegments_in, std::vector<vector<int> > hps_in )
{
	for(unsigned int i=0; i < hps_in.size(); i++)
	{
		IplImage *image_printing;

		image_printing = cvCreateImage( cvGetSize(currentImage_in), IPL_DEPTH_8U, 3 );
		cvSet(image_printing, cvScalar(255, 255, 255));

		CvRNG rng;
		for (unsigned int r=0; r < hps_in[i].size(); r++)
		{

			CvScalar color = random_color(&rng);
			for (unsigned int c=0; c<listSegments_in[abs(hps_in[i][r])].points.size(); c++)
			{				
				if( c != 0 && c!= listSegments_in[abs(hps_in[i][r])].points.size()-1 )
				{
					((uchar *)(image_printing->imageData + (int)listSegments_in[abs(hps_in[i][r])].points[c].y*image_printing->widthStep))[(int)listSegments_in[abs(hps_in[i][r])].points[c].x*image_printing->nChannels + 0]= color.val[0]; // B
					((uchar *)(image_printing->imageData + (int)listSegments_in[abs(hps_in[i][r])].points[c].y*image_printing->widthStep))[(int)listSegments_in[abs(hps_in[i][r])].points[c].x*image_printing->nChannels + 1]= color.val[1]; // G
					((uchar *)(image_printing->imageData + (int)listSegments_in[abs(hps_in[i][r])].points[c].y*image_printing->widthStep))[(int)listSegments_in[abs(hps_in[i][r])].points[c].x*image_printing->nChannels + 2]= color.val[2]; // R
				}
				else
				{
					((uchar *)(image_printing->imageData + (int)listSegments_in[abs(hps_in[i][r])].points[c].y*image_printing->widthStep))[(int)listSegments_in[abs(hps_in[i][r])].points[c].x*image_printing->nChannels + 0]= 0; // B
					((uchar *)(image_printing->imageData + (int)listSegments_in[abs(hps_in[i][r])].points[c].y*image_printing->widthStep))[(int)listSegments_in[abs(hps_in[i][r])].points[c].x*image_printing->nChannels + 1]= 0; // G
					((uchar *)(image_printing->imageData + (int)listSegments_in[abs(hps_in[i][r])].points[c].y*image_printing->widthStep))[(int)listSegments_in[abs(hps_in[i][r])].points[c].x*image_printing->nChannels + 2]= 255; // R
				}
			}
		}
		
		std::stringstream ss;
		std::string imageName;

		ss << fileName_in << "HpsSegments/Frame" << indexFrame_in << "_SegmentHpN" << i <<".jpg";

		imageName = ss.str();
		const char * imageNameChar = imageName.c_str();

		cvSaveImage(imageNameChar, image_printing);

		cvReleaseImage(&image_printing);
	}
}

void printCurveHps( IplImage *currentImage_in, std::string fileName_in, int indexFrame_in, std::vector<trackingHp> trackingHps )
{
	for (unsigned int i=0; i < trackingHps.size(); i++)
	{
		IplImage *image_printing;

		image_printing = cvCreateImage( cvGetSize(currentImage_in), IPL_DEPTH_8U, 0 );
		cvCopy(currentImage_in, image_printing, NULL);

		for (unsigned int c=0; c < trackingHps[i].points.size(); c++)	
			((uchar *)(image_printing->imageData + (int)trackingHps[i].points[c].y*image_printing->widthStep))[(int)trackingHps[i].points[c].x] = 0;

		for (unsigned int c=0; c < trackingHps[i].indexes.size(); c++)
		{	
			cvCircle(image_printing, 
			cvPoint(trackingHps[i].points[trackingHps[i].indexes[c]].x, trackingHps[i].points[trackingHps[i].indexes[c]].y),  
			3,
			cvScalar(0),
			2,
			8);
		}
		
		std::stringstream ss;
		std::string imageName;

		ss << fileName_in << "HpsCurves/Frame" << indexFrame_in << "_CurveHpN" << i <<".jpg";

		imageName = ss.str();
		const char * imageNameChar = imageName.c_str();

		cvSaveImage(imageNameChar, image_printing);

		cvReleaseImage(&image_printing);
	}
}

void printLocalBinaryPatterns( IplImage *currentImage_in, int type_in, std::string fileName_in, int indexFrame_in, trackingHp currentHp_in, std::vector<Point2d> curvePatternCurrentHp_in, std::vector<Point2d> normalPattern1CurrentHp_in, std::vector<Point2d> normalPattern2CurrentHp_in )
{
	IplImage *image_printing;
	image_printing = cvCreateImage( cvGetSize(currentImage_in), IPL_DEPTH_8U, 3 );

	cvSet(image_printing, cvScalar(255, 255, 255));
	cvCopy(currentImage_in, image_printing, NULL);

	for (unsigned int c=0; c  < curvePatternCurrentHp_in.size(); c++)
	{
		Point2d currentPoint = curvePatternCurrentHp_in[c];
		((uchar *)(image_printing->imageData + (int)currentPoint.y*image_printing->widthStep))[(int)currentPoint.x*image_printing->nChannels + 0]= 0;	// B
		((uchar *)(image_printing->imageData + (int)currentPoint.y*image_printing->widthStep))[(int)currentPoint.x*image_printing->nChannels + 1]= 0;	// G
		((uchar *)(image_printing->imageData + (int)currentPoint.y*image_printing->widthStep))[(int)currentPoint.x*image_printing->nChannels + 2]= 255; // R
	}
	for (unsigned int c=0; c  < normalPattern1CurrentHp_in.size(); c++)
	{
		Point2d currentPoint = normalPattern1CurrentHp_in[c];
		((uchar *)(image_printing->imageData + (int)currentPoint.y*image_printing->widthStep))[(int)currentPoint.x*image_printing->nChannels + 0]= 0;	// B
		((uchar *)(image_printing->imageData + (int)currentPoint.y*image_printing->widthStep))[(int)currentPoint.x*image_printing->nChannels + 1]= 255;	// G
		((uchar *)(image_printing->imageData + (int)currentPoint.y*image_printing->widthStep))[(int)currentPoint.x*image_printing->nChannels + 2]= 0; // R
	}
	for (unsigned int c=0; c  < normalPattern2CurrentHp_in.size(); c++)
	{
		Point2d currentPoint = normalPattern2CurrentHp_in[c];
		((uchar *)(image_printing->imageData + (int)currentPoint.y*image_printing->widthStep))[(int)currentPoint.x*image_printing->nChannels + 0]= 255;	// B
		((uchar *)(image_printing->imageData + (int)currentPoint.y*image_printing->widthStep))[(int)currentPoint.x*image_printing->nChannels + 1]= 0;	// G
		((uchar *)(image_printing->imageData + (int)currentPoint.y*image_printing->widthStep))[(int)currentPoint.x*image_printing->nChannels + 2]= 0; // R
	}

	std::stringstream ss;
	std::string imageName;

	if( type_in == 0)
		ss << fileName_in << "LocalBinaryPatterns/Frame" << indexFrame_in << "_Model"  << ".jpg";
	else
		ss << fileName_in << "LocalBinaryPatterns/Frame" << indexFrame_in << "_HpN" << currentHp_in.id << ".jpg";

	imageName = ss.str();
	const char * imageNameChar = imageName.c_str();

	cvSaveImage(imageNameChar, image_printing);

	cvReleaseImage(&image_printing);
}

void printTrackingResult(  IplImage *currentImage_in, std::string fileName_in, int indexFrame_in, std::vector<Point2d> GW_in, int indexBestTrackingHypothesis_in )
{
	IplImage *image_printing1;
	IplImage *image_printing2;

	image_printing1 = cvCreateImage( cvGetSize(currentImage_in), IPL_DEPTH_8U, 3 );
	image_printing2 = cvCreateImage( cvGetSize(currentImage_in), IPL_DEPTH_8U, 3 );
	cvCopy(currentImage_in, image_printing1, NULL);
	cvCopy(currentImage_in, image_printing2, NULL);

	ofstream myfile;
	std::stringstream ss;
	std::string imageName;

	ss << fileName_in << "CatheterPoints/Points_Frame" << indexFrame_in << ".txt";

	imageName = ss.str();
	const char * imageNameChar = imageName.c_str();

	myfile.open(imageNameChar);

	for (unsigned int c=0; c < GW_in.size(); c++)
	{				
		((uchar *)(image_printing1->imageData + (int)GW_in[c].y*image_printing1->widthStep))[(int)GW_in[c].x*image_printing1->nChannels + 0]= 0;	// B
		((uchar *)(image_printing1->imageData + (int)GW_in[c].y*image_printing1->widthStep))[(int)GW_in[c].x*image_printing1->nChannels + 1]= 255;	// G
		((uchar *)(image_printing1->imageData + (int)GW_in[c].y*image_printing1->widthStep))[(int)GW_in[c].x*image_printing1->nChannels + 2]= 0;	// R

		((uchar *)(image_printing2->imageData + (int)GW_in[c].y*image_printing2->widthStep))[(int)GW_in[c].x*image_printing2->nChannels + 0]= 0;	// B
		((uchar *)(image_printing2->imageData + (int)GW_in[c].y*image_printing2->widthStep))[(int)GW_in[c].x*image_printing2->nChannels + 1]= 0;	// G
		((uchar *)(image_printing2->imageData + (int)GW_in[c].y*image_printing2->widthStep))[(int)GW_in[c].x*image_printing2->nChannels + 2]= 255;  // R

		if( c == 0 )
			//(image_printing1, cvPoint(GW_in[c].x, GW_in[c].y), 3, cvScalar(0,0,255), 2, 8);
			cvRectangle(image_printing1, cvPoint(GW_in[c].x-10, GW_in[c].y-10), cvPoint(GW_in[c].x+10, GW_in[c].y+10), cvScalar(0,255,255), 2, 8);
		//else 
			//if ( c == GW_in.size() - 1 )
			//cvCircle(image_printing1, cvPoint(GW_in[c].x, GW_in[c].y), 3, cvScalar(255,0,0), 2, 8);

		myfile << GW_in[c].x << " " << GW_in[c].y << " " << 255 << "\n";
	}

	myfile.close();

	std::stringstream ss1, ss2;
	std::string imageName1, imageName2;

	//ss1 << fileName_in << "TrackingResult/TrackingFrame" << indexFrame_in << "_HpN" << indexBestTrackingHypothesis_in << "_NumFrame" << indexFrame_in << ".jpg";
	ss1 << fileName_in << "TrackingResult/Frame" << indexFrame_in << ".jpg";
	ss2 << fileName_in << "TrackingResult/Frame" << indexFrame_in << ".png";

	imageName1 = ss1.str();
	imageName2 = ss2.str();
	const char * imageNameChar1 = imageName1.c_str();
	const char * imageNameChar2 = imageName2.c_str();

	cvSaveImage(imageNameChar1, image_printing1);
	cvSaveImage(imageNameChar2, image_printing2);

	//cvShowImage("Tracking results", image_printing2);
    //cvShowImage("Tracking results", image_printing1);

    //cvWaitKey(1);

	cvReleaseImage(&image_printing1);
	cvReleaseImage(&image_printing2);
}

void printGWModel( IplImage *currentImage_in, std::string fileName_in, int indexFrame_in, trackingHp model_in )
{
	IplImage *image_printing;

	image_printing = cvCreateImage( cvGetSize(currentImage_in), IPL_DEPTH_8U, 0 );
	cvSet(image_printing, cvScalar(255, 255, 255));
	cvCopy(currentImage_in, image_printing, NULL);

	for (unsigned int c=0; c  < model_in.points.size(); c++)
		((uchar *)(image_printing->imageData + (int)model_in.points[c].y*image_printing->widthStep))[(int)model_in.points[c].x] = 0;

	for (unsigned int c=0; c < model_in.indexes.size(); c++)
		cvCircle(image_printing, cvPoint(model_in.points[model_in.indexes[c]].x, model_in.points[model_in.indexes[c]].y), 3, cvScalar(0), 2, 8);

	std::stringstream ss;
	std::string imageName;

	ss << fileName_in << "HpsCurves/Model_Frame" << indexFrame_in << ".jpg";

	imageName = ss.str();
	const char * imageNameChar = imageName.c_str();

	cvSaveImage(imageNameChar, image_printing);
	cvReleaseImage(&image_printing);
}

void printFirstFrame( IplImage *currentImage_in, std::string fileName_in, std::vector<Point2d> GW_in )
{
	IplImage *image_printing1;
	image_printing1 = cvCreateImage( cvGetSize(currentImage_in), IPL_DEPTH_8U, 3 );
	cvCopy(currentImage_in, image_printing1, NULL);

	for (unsigned int c=0; c < GW_in.size(); c++)
	{				
		((uchar *)(image_printing1->imageData + (int)GW_in[c].y*image_printing1->widthStep))[(int)GW_in[c].x*image_printing1->nChannels + 0]= 0;	// B
		((uchar *)(image_printing1->imageData + (int)GW_in[c].y*image_printing1->widthStep))[(int)GW_in[c].x*image_printing1->nChannels + 1]= 255;	// G
		((uchar *)(image_printing1->imageData + (int)GW_in[c].y*image_printing1->widthStep))[(int)GW_in[c].x*image_printing1->nChannels + 2]= 0; // R

		if( c == 0 )
			cvCircle(image_printing1, cvPoint(GW_in[c].x, GW_in[c].y), 2, cvScalar(0,0,255), 2, 8);
		else if ( c == GW_in.size() - 1 )
			cvCircle(image_printing1, cvPoint(GW_in[c].x, GW_in[c].y), 2, cvScalar(255,0,0), 2, 8);
	}

    //cvShowImage("First Frame", image_printing1);
    //cvWaitKey(1);

	std::stringstream ss1;
	std::string imageName1;

	ss1 << fileName_in << "firstFrame.png";

	imageName1 = ss1.str();
	const char * imageNameChar1 = imageName1.c_str();
	cvSaveImage(imageNameChar1, image_printing1);
	cvReleaseImage(&image_printing1);
}

