#include "tracker_general.h"

using namespace dvrk;

TrackerGeneral::TrackerGeneral(cv::Size _pattern_size, cv::Size _roi_size,
                             cv::SimpleBlobDetector::Params params,
                             cv::SimpleBlobDetector::Params params_roi) :
    last_valid_location(cv::Point2f(200, 200)),
    pattern_size(_pattern_size), square_size (1.0f),
    roi_hw(_roi_size.width), roi_hh(_roi_size.height),
    binitTracker(false), bisTracking(false)

{
    //blob_detector = new cv::SimpleBlobDetector(params);
    cv::SimpleBlobDetector temp;
    temp.create(params);
    blob_detector = &temp;

    //roi_blob_detector = new cv::SimpleBlobDetector(params_roi);
    cv::SimpleBlobDetector temp2;
    temp2.create(params);
    roi_blob_detector = &temp2;

    // record model points on the grid
    for( int i = 0; i < pattern_size.height; i++ )
    {
        for( int j = 0; j < pattern_size.width; j++ )
        {
            cv::Point2f idealPt;
            idealPt = cv::Point2f((2*j + i % 2)*square_size, i*square_size);
            model_dots.push_back(idealPt);
        }
    }

    // 3-----0
    // |*****|
    // 2-----1
    corner_pts.push_back(cv::Point2f(0, 0));
    corner_pts.push_back(cv::Point2f((pattern_size.width*2-1)*square_size, 0));
    corner_pts.push_back(cv::Point2f((pattern_size.width*2-1)*square_size, (pattern_size.height-1)*square_size));
    corner_pts.push_back(cv::Point2f(0, (pattern_size.height-1)*square_size));

    // generate colors for identity
    //////////////////////////////////////////////////////////////////////////
    cv::RNG rng(12345);
    for (int i = 0; i < pattern_size.height; i++)
    {
        cv::Scalar color = cv::Scalar(rng.uniform(100, 255), rng.uniform(100, 255),rng.uniform(100, 255)); //unknow color
        for (int j = 0; j < pattern_size.width; j++)
            dot_colors.push_back(color);
    }

}

TrackerGeneral::TrackerGeneral(std::vector<cv::Point2f> modelDef)
{
    square_size =1.0f;
    for (int i=0; i<modelDef.size(); i++)
    {
        model_dots.push_back(modelDef[i]);
    }
    cnt_notdetected = 0;
}

bool TrackerGeneral::track(const cv::Mat &cur_image, std::vector<cv::Point2f> &curr_dots, bool found)
{
	cv::Mat cur_gray;
	cv::cvtColor(cur_image, cur_gray, cv::COLOR_BGR2GRAY);

    //bool found = DetectPattern(cur_gray, curr_dots);

	if (found)
	{
		std::vector<unsigned char> inls;
		homography = cv::findHomography(model_dots, curr_dots, inls, CV_RANSAC, square_size*0.8); 
		bisTracking = false;
		if (!binitTracker)
		{
			initTrack(model_dots, cur_gray, curr_dots);
		}

		// Once pattern is detected, update tracking for next image
        cnt_notdetected = 0;
		UpdateStatus();
	}
	else // If detection is not good -> perform tracking using optical flow
	{
        if (cnt_notdetected < 10)
        {
            std::vector<cv::Point2f> cur_pts;
            bool homography_valid = TrackPattern(cur_gray, cur_pts, homography);


            if (homography_valid)
            {
                std::vector<cv::Point2f> h_pts;
                cv::perspectiveTransform(model_dots, h_pts, homography);
                //std::cout<<h_pts.size()<<std::endl;
                curr_dots = h_pts;
                bisTracking = true;
                cnt_notdetected ++;
                //std::cout << "curr_dots " << curr_dots[0] << std::endl;

            }
            else
                return false;
        }
        else
        {
            cnt_notdetected ++;
            return false;
        }
	}

	// Update tracking and detection for next image
	if (binitTracker)
		UpdateLastDots(cur_gray, curr_dots);
	UpdateLastLocation(curr_dots);
	return true;
}

bool TrackerGeneral::DetectPattern(const cv::Mat& _img_gray, std::vector<cv::Point2f>& _dots)
{
	// Two attempts
	bool found = FindDots(_img_gray, pattern_size, _dots, 
		cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blob_detector);
	if (!found)
	{
		// crop a roi for detection
		cv::Rect rect = cv::Rect((int)last_valid_location.x, (int)last_valid_location.y, 
			2*roi_hw, 2*roi_hh);
		rect.x = rect.x < 0? 0 : rect.x;
		rect.y = rect.y < 0? 0 : rect.y;
		rect.width = (rect.x+rect.width) 
		> (_img_gray.cols - 1)? (_img_gray.cols-rect.x-1) : rect.width;
		rect.height = (rect.y+rect.height) 
		> (_img_gray.rows - 1)? (_img_gray.rows-rect.y-1) : rect.height;
		//cv::rectangle(cur_image, rect, CV_RGB(250,250, 0));
		cv::Mat roi = _img_gray(rect);
		found = FindDots(roi, pattern_size, _dots, 
			cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, roi_blob_detector);
		if (found)
		{
			for (unsigned int i = 0; i < _dots.size(); i++)
			{
				_dots[i].x = _dots[i].x + last_valid_location.x;
				_dots[i].y = _dots[i].y + last_valid_location.y;
			}
		}
	}
	return found;
}

bool TrackerGeneral::FindDots(cv::InputArray _image, cv::Size patternSize, cv::OutputArray _centers,
								int flags, const cv::Ptr<cv::FeatureDetector> &blobDetector)
{
	bool isAsymmetricGrid = (flags & cv::CALIB_CB_ASYMMETRIC_GRID) ? true : false;
	bool isSymmetricGrid  = (flags & cv::CALIB_CB_SYMMETRIC_GRID ) ? true : false;
	CV_Assert(isAsymmetricGrid ^ isSymmetricGrid);

	cv::Mat image = _image.getMat();
	std::vector<cv::Point2f> centers;

	std::vector<cv::KeyPoint> keypoints;
	blobDetector->detect(image, keypoints);
	std::vector<cv::Point2f> points;
	for (size_t i = 0; i < keypoints.size(); i++)
	{
		points.push_back(keypoints[i].pt);
	}
	//for (int i = 0; i < status.size(); i++)
	//{
	//	if (status[i])
	//	{
	//		points.push_back(cur_pts[i]);
	//	}
	//}
	//std::cout<<points.size()<<std::endl;
	if(flags & cv::CALIB_CB_CLUSTERING)
	{
		CirclesGridClusterFinder circlesGridClusterFinder(isAsymmetricGrid);
        circlesGridClusterFinder.findGrid(points, patternSize, centers);
        //circlesGridClusterFinder.findGrid(points, patternSize, centers);
		cv::Mat(centers).copyTo(_centers);
		return !centers.empty();
	}

    myCirclesGridFinderParameters parameters;
	parameters.vertexPenalty = -0.6f;
	parameters.vertexGain = 1;
	parameters.existingVertexGain = 10000;
	parameters.edgeGain = 1;
	parameters.edgePenalty = -0.6f;

	if(flags & cv::CALIB_CB_ASYMMETRIC_GRID)
        parameters.gridType = myCirclesGridFinderParameters::ASYMMETRIC_GRID;
	if(flags & cv::CALIB_CB_SYMMETRIC_GRID)
        parameters.gridType = myCirclesGridFinderParameters::SYMMETRIC_GRID;

	const int attempts = 2;
	const size_t minHomographyPoints = 4;
	cv::Mat H;
	for (int i = 0; i < attempts; i++)
	{
		centers.clear();
		CirclesGridFinder boxFinder(patternSize, points, parameters);
		bool isFound = false;

		try
		{
			isFound = boxFinder.findHoles();
		}
		catch (cv::Exception)
		{

		}

		if (isFound)
		{
			switch(parameters.gridType)
			{
            case myCirclesGridFinderParameters::SYMMETRIC_GRID:
				boxFinder.getHoles(centers);
				break;
            case myCirclesGridFinderParameters::ASYMMETRIC_GRID:
				boxFinder.getAsymmetricHoles(centers);
				break;
			default:
				CV_Error(CV_StsBadArg, "Unkown pattern type");
			}

			if (i != 0)
			{
				cv::Mat orgPointsMat;
				cv::transform(centers, orgPointsMat, H.inv());
				cv::convertPointsFromHomogeneous(orgPointsMat, centers);
			}
			cv::Mat(centers).copyTo(_centers);
			return true;
		}

		boxFinder.getHoles(centers);
		if (i != attempts - 1)
		{
			if (centers.size() < minHomographyPoints)
				break;
			H = CirclesGridFinder::rectifyGrid(boxFinder.getDetectedGridSize(), centers, points, points);
		}
	}
	cv::Mat(centers).copyTo(_centers);
	return false;
}

void TrackerGeneral::UpdateLastLocation(const std::vector<cv::Point2f>& _dots)
{
	if (_dots.size()>0)
	{
		cv::Point2f pt(0,0);
		for (unsigned int i = 0; i < _dots.size(); i++)
		{
			pt += _dots[i];
		}
		last_valid_location.x = static_cast<float>(cvRound(pt.x/_dots.size() - roi_hw));
		last_valid_location.y = static_cast<float>(cvRound(pt.y/_dots.size() - roi_hh));
	}
}





/************************************************************************/
/* Tracking part                                                        */
/************************************************************************/

void TrackerGeneral::initTrack(std::vector<cv::Point2f> _model_dots, cv::Mat& _pre_gray,
	std::vector<cv::Point2f> _prev_dots)
{
	pre_gray = _pre_gray.clone();
	model_dots = _model_dots;
	prev_dots = _prev_dots;
	pre_status.resize(model_dots.size());
	std::fill(pre_status.begin(), pre_status.end(), 1);
	binitTracker = true;
}

bool TrackerGeneral::TrackPattern(const cv::Mat& _cur_gray, std::vector<cv::Point2f>& _dots, cv::Mat& _H)
{
	bool homography_valid = false;
	if (binitTracker && prev_dots.size() > 0)
	{
		_dots.clear();
		std::vector<unsigned char> status;
		DoSpaseOpticalFlow(pre_gray, _cur_gray, prev_dots, _dots, status);
		std::vector<cv::Point2f> mod_pts;
		std::vector<cv::Point2f> dsc_pts;
		for (unsigned int i = 0; i < pre_status.size(); i++)
		{
			if (!pre_status[i] && status[i])
			{
				status[i] = 0;
			}
			if (pre_status[i] && status[i])
			{
				mod_pts.push_back(model_dots[i]);
				dsc_pts.push_back(_dots[i]);
			}
		}
		//std::cout<<dsc_pts.size()<<std::endl;
		pre_status = status;

		std::vector<unsigned char> inliers;
		
		if(mod_pts.size()>=4 && dsc_pts.size()>=4){
			//_H = cv::findHomography(model_dots, _dots, inliers, CV_RANSAC, 0.8);
            _H = cv::findHomography(mod_pts, dsc_pts, inliers, CV_RANSAC, 2.0);
		}
		else
			return false;
		

		double count = 0;
		for (unsigned int i = 0; i < inliers.size(); i++)
		{
			if (inliers[i])
			{
				count = count + 1.;
			}
		}
		if (count/status.size() > 0.5)
			homography_valid = true;
	}

	return homography_valid;
}

void TrackerGeneral::DoSpaseOpticalFlow(const cv::Mat& _prev_img, const cv::Mat& _cur_img,
	const std::vector<cv::Point2f>& _prev_pts, std::vector<cv::Point2f>& _cur_pts, 
	std::vector<unsigned char>& _status)
{
	std::vector<float> errs;
	cv::calcOpticalFlowPyrLK(_prev_img, _cur_img, _prev_pts, _cur_pts, _status, errs);
}

void TrackerGeneral::UpdateLastDots(cv::Mat& _cur_gray, std::vector<cv::Point2f> _prev_dots)
{
	pre_gray = _cur_gray.clone();
	prev_dots = _prev_dots;
}

void TrackerGeneral::UpdateStatus()
{
	std::fill(pre_status.begin(), pre_status.end(), 1);
}

void TrackerGeneral::drawKeydots(cv::InputOutputArray _image)
{
	cv::Mat image = _image.getMat();
	cv::Scalar bgr;
	bgr = bisTracking ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 255, 0);

	cv::perspectiveTransform(corner_pts, curr_corners, homography);

	
	for (unsigned int i = 0; i < curr_dots.size(); i++)
	{
		cv::circle(image, curr_dots[i], 4, dot_colors[i], 1, CV_AA);
		cv::line(image, cv::Point2f(curr_dots[i].x-3, curr_dots[i].y), 
			cv::Point2f(curr_dots[i].x+3, curr_dots[i].y), dot_colors[i]);
		cv::line(image,cv::Point2f( curr_dots[i].x, curr_dots[i].y-3),
			cv::Point2f(curr_dots[i].x, curr_dots[i].y+3), dot_colors[i] );
	}

	cv::line(image, curr_corners[0], curr_corners[1], bgr, 2, CV_AA);
	cv::line(image, curr_corners[1], curr_corners[2], bgr, 2, CV_AA);
	cv::line(image, curr_corners[2], curr_corners[3], bgr, 2, CV_AA);
	cv::line(image, curr_corners[0], curr_corners[3], bgr, 2, CV_AA);

}

std::vector<cv::Point2f> TrackerGeneral::get_corners() {
	cv::perspectiveTransform(corner_pts, curr_corners, homography);
	return curr_corners;
}
