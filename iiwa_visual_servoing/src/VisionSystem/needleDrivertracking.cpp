#include"needleDrivertracking.h"

NeedleDriverTracking::NeedleDriverTracking()
{
    FileStorage f_P1("../source/VisionSystem/caliInfo/steroCaliOutput/P1.xml", FileStorage::READ); f_P1["P1"]>>P1;
    FileStorage f_P2("../source/VisionSystem/caliInfo/steroCaliOutput/P2.xml", FileStorage::READ); f_P2["P2"]>>P2;

    //intial keydot parameter
    nx = 4;     ny = 11;
    keydot_square_size = 9.676/2.0/1000;


    cv::Size pattern_size(nx, ny); // width, height
    // Global blob detector setting
    cv::SimpleBlobDetector::Params params;
    params.minRepeatability = 2;
    params.minDistBetweenBlobs = 2;
    params.thresholdStep = 20;
    params.minArea = 1;
    params.maxArea = 500;
    params.minThreshold = 10;
    params.maxThreshold = 220;
    params.filterByArea = true;

    // Local blob detector setting
    cv::SimpleBlobDetector::Params params_roi;
    params_roi.minRepeatability = 2;
    params_roi.minDistBetweenBlobs = 5;
    params_roi.thresholdStep = 10;
    params_roi.minArea = 5;
    params_roi.maxArea = 400;
    params_roi.minThreshold = 10;
    params_roi.maxThreshold = 220;
    params_roi.filterByArea = true;

    cv::Size roi_size (100,100);

}

bool NeedleDriverTracking::KeyDotsObserved( stereoImages & Images, vector<cv::Point2f> &cornersLeft, vector<cv::Point2f> &cornersRight)
{
    cv::SimpleBlobDetector::Params params;
    params.minRepeatability = 2;
    params.minDistBetweenBlobs = 2;
    params.thresholdStep = 50;
    params.minArea = 5;
    params.maxArea = 300;
    params.minThreshold = 10;
    params.maxThreshold = 220;
    params.blobColor = 0 ;

    cv::SimpleBlobDetector test;
    //cv::Ptr<cv::SimpleBlobDetector> blobDetector = new cv::SimpleBlobDetector(params);
    cv::SimpleBlobDetector blobDetector ; //= new cv::SimpleBlobDetector(params);
    blobDetector.create(params);

    cv::Size keydot_pattern_size(nx, ny);

    cv::Mat gray0;
    cv::cvtColor(Images.frameLeft, gray0, CV_BGR2GRAY);

    bool result1 = cv::findCirclesGrid(gray0, keydot_pattern_size, cornersLeft,
        cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, &blobDetector);

    cv::Mat gray1;
    cv::cvtColor(Images.frameRight, gray1, CV_BGR2GRAY);

    bool result2 = cv::findCirclesGrid(gray1, keydot_pattern_size, cornersRight,
        cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, &blobDetector);

    if ( result1==true &&  result2==true )
    {
        return true;
    }
    else
    {
        return false;
    }
}

stereoImages NeedleDriverTracking::drawKeyDotsOnStereo(stereoImages &Images, vector<cv::Point2f> &pointsLeft, vector<cv::Point2f> &pointsRight)
{
    stereoImages imageTemp= Images;

   // if (KeyDotsObserved  ( imageTemp,  pointsLeft, pointsRight )  )
    {      
        for (int i = 0; i < pointsLeft.size(); i++)
        {
            cv::circle(Images.frameLeft,  pointsLeft[i], 6,  CV_RGB(255, 0 ,0),4);
            cv::circle(Images.frameRight, pointsRight[i], 6, CV_RGB(255, 0 ,0),4);
        }

        cv::circle(Images.frameLeft,  pointsLeft[0], 6,  CV_RGB(0, 0 ,255),4);
        cv::circle(Images.frameRight, pointsRight[0], 6, CV_RGB(0, 0 ,255),4);
    } 
    return imageTemp;
}

bool NeedleDriverTracking::findPatternPose( vector<cv::Point2f> &pointsLeft, vector<cv::Point2f> &pointsRight, cv::Mat & poseCameraFrame)
{
//    //------------ Write dots to file -----------
//    ofstream fileDotsInCamera;
//    fileDotsInCamera.open("../source/VisionSystem/caliInfo/handEyeOutput/dotsInCamera.txt" );

    //------------ find Keydots -----------------
//    std::vector<cv::Point2f> pointsLeft;
//    std::vector<cv::Point2f> pointsRight;

    cv::Size keydot_pattern_size(nx, ny);

//    bool sta=KeyDotsObserved(Images, pointsLeft, pointsRight) ;


        // triangulate
        std::vector<cv::Point3f> objectPointsCameraFrame;
        Mat points4D;
        triangulatePoints(P1, P2, Mat(pointsLeft), Mat(pointsRight), points4D);
        //cout << "number of dots found: " << points4D.cols <<endl;
//        cout << points4D << endl;
        for (int i=0; i<points4D.cols; i++)
        {
//            fileDotsInCamera << points4D.at<float>(0,i)/points4D.at<float>(3,i) << " ";
//            fileDotsInCamera << points4D.at<float>(1,i)/points4D.at<float>(3,i) << " ";
//            fileDotsInCamera << points4D.at<float>(2,i)/points4D.at<float>(3,i) << endl;
            objectPointsCameraFrame.push_back(cv::Point3f ( points4D.at<float>(0,i)/points4D.at<float>(3,i),
                                                            points4D.at<float>(1,i)/points4D.at<float>(3,i),
                                                            points4D.at<float>(2,i)/points4D.at<float>(3,i) ) );
        }

//        cout << "Point 3D: " << objectPointsCameraFrame[0] << endl;

        std::vector<cv::Point3f> objectPointsLocal;
        for( int i = 0; i < keydot_pattern_size.height; i++ )
        for( int j = 0; j < keydot_pattern_size.width; j++ )
        {
            objectPointsLocal.push_back(cv::Point3f(float((2*j + i % 2)*keydot_square_size), float(i*keydot_square_size), 0));
        }
        //camera frame points

        //for( int i = 0; i < keydot_pattern_size.height; i++ )
        //for( int j = 0; j < keydot_pattern_size.width; j++ )
        //{

            //objectPointsCameraFrame.push_back(cv::Point3f ( points4D.at<float>(0,2*j + i % 2)/points4D.at<float>(3,2*j + i % 2),   points4D.at<float>(1,2*j + i % 2)/points4D.at<float>(3,2*j + i % 2)  ,points4D.at<float>(2,2*j + i % 2)/points4D.at<float>(3,2*j + i % 2) ) );
        //cout<<points4D.at<float>(0,2*j + i % 2)/points4D.at<float>(3,2*j + i % 2)<<"  "<<   points4D.at<float>(1,2*j + i % 2)/points4D.at<float>(3,2*j + i % 2)<<"  "<<points4D.at<float>(2,2*j + i % 2)/points4D.at<float>(3,2*j + i % 2) <<endl;
        //}
//        cout<<"point4D: " <<points4D.at<float>(0,0)/points4D.at<float>(3,0)/1000<<"  "<<   points4D.at<float>(1,0)/points4D.at<float>(3,0)/1000<<"  "<<points4D.at<float>(2,0)/points4D.at<float>(3,0)/1000 <<endl;

        //pose estimation
        cv:Mat camerInMarkerFrame, camerInMarkerFrame_abs;
        camerInMarkerFrame_abs = cv::Mat::eye(4,4,CV_64FC1);
        //find_rigidTransformation(objectPointsLocal,objectPointsCameraFrame,camerInMarkerFrame);


        std::vector<Point3D> left(objectPointsLocal.size()), right(objectPointsCameraFrame.size());
        for (int i=0; i<left.size(); i++)
        {
            left[i][0] = double(objectPointsLocal[i].x);
            left[i][1] = double(objectPointsLocal[i].y);
            left[i][2] = double(objectPointsLocal[i].z);

            right[i][0] = double(objectPointsCameraFrame[i].x);
            right[i][1] = double(objectPointsCameraFrame[i].y);
            right[i][2] = double(objectPointsCameraFrame[i].z);
        }
        double norml, normr;
        norml = sqrt((left[0][0] - left[1][0]) * (left[0][0] - left[1][0]) +
                     (left[0][1] - left[1][1]) * (left[0][1] - left[1][1]) +
                     (left[0][2] - left[1][2]) * (left[0][2] - left[1][2]) );

        normr = sqrt((right[0][0] - right[1][0]) * (right[0][0] - right[1][0]) +
                     (right[0][1] - right[1][1]) * (right[0][1] - right[1][1]) +
                     (right[0][2] - right[1][2]) * (right[0][2] - right[1][2]) );

    if (sqrt((norml-normr)*(norml-normr))<0.0005)
    {
        // if error is smaller than 1mm
//         cout << "left: " << norml << endl;
//         cout << "right: " << normr << endl;

        Frame computedTransformation;
        AbsoluteOrientation::compute(left,right,computedTransformation);
        computedTransformation.getRotationMatrix(camerInMarkerFrame_abs.at<double>(0,0), camerInMarkerFrame_abs.at<double>(0,1), camerInMarkerFrame_abs.at<double>(0,2),
                                                 camerInMarkerFrame_abs.at<double>(1,0), camerInMarkerFrame_abs.at<double>(1,1), camerInMarkerFrame_abs.at<double>(1,2),
                                                 camerInMarkerFrame_abs.at<double>(2,0), camerInMarkerFrame_abs.at<double>(2,1), camerInMarkerFrame_abs.at<double>(2,2));
        computedTransformation.getTranslation(camerInMarkerFrame_abs.at<double>(0,3),camerInMarkerFrame_abs.at<double>(1,3),camerInMarkerFrame_abs.at<double>(2,3));

//        cout << "rigid: " << endl;
//        cout << camerInMarkerFrame << endl;
//        cout << "abs: " << endl;
//        cout << camerInMarkerFrame_abs << endl;

        poseCameraFrame=camerInMarkerFrame_abs;

        for (int i=0;i<3;i++)
        {
           poseCameraFrame.at<double>(i,3)= poseCameraFrame.at<double>(i,3); ///1000.;
        }
        //cout<< "poseCameraFrame :" << poseCameraFrame.at<double>(0,3) << " " << poseCameraFrame.at<double>(1,3) << " " << poseCameraFrame.at<double>(2,3)<<endl;

        return true;
    }
    else
    {
        // if error is larger than 0.5mm
        cout << "Error larger than 0.5mm!" << endl;
        return false;
    }

}



