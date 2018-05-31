#include "tracker_pattern.h"


using namespace dvrk;

Tracker_Pattern::Tracker_Pattern() : pattern_size(4, 11), pattern_type(DOT_PATTERN)
{
	cv::SimpleBlobDetector::Params params;
	params.minRepeatability = 2;
	params.minDistBetweenBlobs = 5;
	params.thresholdStep = 30;
	params.minArea = 25;
	params.maxArea = 5000;
	params.minThreshold = 10;
    params.maxThreshold = 220;

    cv::SimpleBlobDetector temp;
    temp.create(params);
    blobDetector = &temp;
    //blobDetector = new cv::SimpleBlobDetector.create(params);
}

Tracker_Pattern::Tracker_Pattern(const cv::Size &pat_size, PatternType pat_type)
	: pattern_size(pat_size), pattern_type(pat_type)
{
	cv::SimpleBlobDetector::Params params;
	params.minRepeatability = 2;
	params.minDistBetweenBlobs = 5;
	params.thresholdStep = 30;
	params.minArea = 10;///25;
	params.maxArea = 5000;
	params.minThreshold = 10;
    params.maxThreshold = 220;

    cv::SimpleBlobDetector temp;
    temp.create(params);
    blobDetector = &temp;
    //blobDetector = new cv::SimpleBlobDetector(params);
}

void Tracker_Pattern::initTrack(const cv::Mat &I)
{
	cv::Mat gray;
	if(I.channels() != 1)
		cv::cvtColor(I, gray, cv::COLOR_BGR2GRAY);

	switch (pattern_type)
	{
	case DOT_PATTERN:
		cv::findCirclesGrid(gray, pattern_size, P_img, cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetector);
		break;

	case CHESS_PATTERN:
		bool found = cv::findChessboardCorners(gray, pattern_size, P_img,
			cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
		if (found)
		{
			cv::cornerSubPix(gray, P_img, cv::Size(11, 11), cv::Size(-1, -1),
				cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
		}
		break;
	}
}

bool Tracker_Pattern::track(const cv::Mat &I)
{
	cv::Mat gray;
	bool found(false);
	if(I.channels() != 1)
		cv::cvtColor(I, gray, cv::COLOR_BGR2GRAY);

	switch (pattern_type)
	{
	case DOT_PATTERN:
		found = cv::findCirclesGrid(gray, pattern_size, P_img, cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetector);
		break;

	case CHESS_PATTERN:
		found = cv::findChessboardCorners(gray, pattern_size, P_img,
			cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
		if (found)
		{
			cv::cornerSubPix(gray, P_img, cv::Size(11, 11), cv::Size(-1, -1),
				cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
		}
		break;
	}
	return found;
}


std::vector<cv::Point2f> Tracker_Pattern::getP_img()
{
	//std::vector<cv::Point2f> dst;
	//cv::Mat(P_img).copyTo(dst);
	return P_img;
}

// std::vector<cv::Point2d> dvrkPatTrack::getP_norm(
// 	const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs)
// {
// 	std::vector<cv::Point2d> Pnorm;
// 	for (unsigned int i = 0; i < P_img.size(); i++)
// 	{
// 		double x, y;
// 		dvrkPixelMeterConversion::convertPointWithDistortion(
// 			cameraMatrix, distCoeffs, 
// 			P_img[i].x, P_img[i].y,
// 			x, y);
// 		Pnorm.push_back(cv::Point2d(x, y));
// 	}
// 	return Pnorm;
// }
