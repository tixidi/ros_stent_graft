#include<VisionSystem/needleTracking.h>

NeedlePoseEstimation::NeedlePoseEstimation()
{
    int startingPointInstrument = 1;		// BaseGW: 0 Y=MAX; 1 Y=MIN;
    int verbose					= 1;
    double sigmaHessian			= 2.0;
    std::string seqName1_in("Data/output_left/");
    std::string seqName2_in("Data/output_right/");

    needleDectionLeft = GWTracking(seqName1_in, 0, startingPointInstrument, verbose, sigmaHessian);
    needleDectionRight = GWTracking(seqName2_in, 0, startingPointInstrument, verbose, sigmaHessian);
}

bool NeedlePoseEstimation::doPoseEstimation(cv::Mat leftImage, cv::Mat rightImage, cv::Mat &estNeedlePose, std::vector<cv::Point3d> &needle3Dmodel, cv::Mat &needleDriverPose, cv::Mat &finalNeedlePose)
{
    Mat leftImage_gray;
    Mat rightImage_gray;

    cvtColor(leftImage, leftImage_gray, CV_RGB2GRAY);
    cvtColor(rightImage, rightImage_gray, CV_RGB2GRAY);

    IplImage currFrame3cLeft;
    IplImage currFrame1cLeft;
    IplImage currFrame3cRight;
    IplImage currFrame1cRight;

    currFrame3cLeft = leftImage;
    currFrame3cRight = rightImage;
    currFrame1cLeft = leftImage_gray;
    currFrame1cRight = rightImage_gray;

    //Compute from know 3D position the 2D projections of the needle in the images for initialization
    std::vector<Point2d> needlePointsLeft_estimate;
    std::vector<Point2d> needlePointsRight_estimate;

    project3DModel( leftImage, rightImage, needle3Dmodel, needlePointsLeft_estimate, needlePointsRight_estimate);

    //Dection of the needle in left and right images
    std::vector<Point2d> needlePointsLeft_final;
    std::vector<Point2d> needlePointsRight_final;

    needleDectionLeft.initialization(needlePointsLeft_estimate, &currFrame3cLeft, &currFrame1cLeft);
    needleDectionRight.initialization(needlePointsRight_estimate, &currFrame3cRight, &currFrame1cRight);

    needleDectionLeft.runTracking(1, &currFrame3cLeft, &currFrame1cLeft, needlePointsLeft_final);
    needleDectionRight.runTracking(1, &currFrame3cRight, &currFrame1cRight, needlePointsRight_final);

    std::vector<cv::Point3d> needlePoints_final;

    cout << "Needle Detection Done \n"<<endl;

    needleReconstruction3D(leftImage, rightImage, needlePointsLeft_final, needlePointsRight_final, needlePoints_final, leftImage.cols);
    
    cout << "Needle 3D Reconstruction Done \n"<<endl;

    poseEstimation3D(needle3Dmodel, needleDriverPose, needlePoints_final, finalNeedlePose);

    cout << "Needle Pose Estimation Done \n"<<endl;

    cout << "finalNeedlePose " << finalNeedlePose << endl;

    project3DModel2( leftImage, rightImage, needle3Dmodel, finalNeedlePose, needlePoints_final);

    //cvWaitKey();
}

bool NeedlePoseEstimation::needleReconstruction3D(cv::Mat leftImage, cv::Mat rightImage, std::vector<cv::Point2d> &needlePointsLeft, std::vector<cv::Point2d> &needlePointsRight, std::vector<cv::Point3d> &needlePoints, int width_image)
{
    std::vector<cv::Point2f> needlePointsLeft_candidates;
    std::vector<cv::Point2f> needlePointsRight_candidates;

    std::vector<cv::Point3f> epipolarLines;
    std::vector<cv::Point2f> needlePointsLeft3f;

    for(int i=0; i<needlePointsLeft.size(); i++)
    {
        Point2f currentPoint;

        currentPoint.x = needlePointsLeft[i].x;
        currentPoint.y = needlePointsLeft[i].y;

        needlePointsLeft3f.push_back(currentPoint);
    }

    Mat points4D;
    Mat P1,P2,F,D1,D2;
    FileStorage f_P1(CAMERA_CALIB_DIR + "P1.xml", FileStorage::READ); f_P1["P1"]>>P1;
    FileStorage f_P2(CAMERA_CALIB_DIR + "P2.xml", FileStorage::READ); f_P2["P2"]>>P2;

    FileStorage f_F(CAMERA_CALIB_DIR + "F.xml", FileStorage::READ); f_F["F"]>>F;
    
    FileStorage f_D1(CAMERA_CALIB_DIR + "D1.xml", FileStorage::READ); f_D1["D1"]>>D1;
    FileStorage f_D2(CAMERA_CALIB_DIR + "D2.xml", FileStorage::READ); f_D2["D2"]>>D2;

    //computeCorrespondEpilines(needlePointsLeft, 1, F, epipolarLines);
    //computeCorrespondEpilines(needlePointsLeft3f, 1, F, epipolarLines);
    
    //Find corrispondences between left and right needle detections. Only one possible match is assumed
    for(int i=0; i<needlePointsLeft.size(); i++)
    {
        cv::Point2f currentPoint = needlePointsLeft[i];
        //cv::Point3d currentLine = epipolarLines[i];

        for(int x=0; x<width_image; x++)
        {
            cv::Point2f currentPointCandidate;

            currentPointCandidate.x = x;
            //currentPointCandidate.y = ((-1*currentLine.z) - (currentLine.x*x))/currentLine.y;
            currentPointCandidate.y = currentPoint.y;

            double minDistance = DBL_MAX;
            for(int j=0; j<needlePointsRight.size(); j++)
            {
                double currentDistance = sqrt(pow(currentPointCandidate.x-needlePointsRight[j].x,2) + pow(currentPointCandidate.y-needlePointsRight[j].y,2));

                if(minDistance > currentDistance)
                    minDistance = currentDistance;
            }

            if(minDistance <= 1.0)
            {
				needlePointsLeft_candidates.push_back(currentPoint);
                needlePointsRight_candidates.push_back(currentPointCandidate);
                break;
            }
        }
    }


    /*for(int i=0; i<needlePointsLeft_candidates.size(); i++)
    {
        Vec3b intensity;
        intensity.val[0] = 0;
        intensity.val[1] = 255;
        intensity.val[2] = 0;
        leftImage.at<Vec3b>(needlePointsLeft_candidates[i].y, needlePointsLeft_candidates[i].x) = intensity;
    }

    imshow("frame left",leftImage);
    cvWaitKey();*/

    /*for(int i=0; i<needlePointsRight_candidates.size(); i++)
    {
        Vec3b intensity;
        intensity.val[0] = 0;
        intensity.val[1] = 255;
        intensity.val[2] = 0;
        rightImage.at<Vec3b>(needlePointsRight_candidates[i].y, needlePointsRight_candidates[i].x) = intensity;
    }

    imshow("frame left",rightImage);
    cvWaitKey();*/

    /*std::vector<cv::Point2f> needlePointsLeft_candidates_undistorted;
    std::vector<cv::Point2f> needlePointsRight_candidates_undistorted;

    undistortPoints(needlePointsLeft_candidates, needlePointsLeft_candidates_undistorted, M1, D1, Mat(), M1);
    undistortPoints(needlePointsRight_candidates, needlePointsRight_candidates_undistorted, M2, D2, Mat(), M2);*/

    // Triangulate
    triangulatePoints(P1, P2, Mat(needlePointsLeft_candidates), Mat(needlePointsRight_candidates), points4D);

    std::vector<cv::Point3f> needlePointsTmp;
    for(int i=0; i<points4D.cols; i++)
    {
        needlePointsTmp.push_back(cv::Point3f ( points4D.at<float>(0,i)/points4D.at<float>(3,i),
                                                        points4D.at<float>(1,i)/points4D.at<float>(3,i),
                                                        points4D.at<float>(2,i)/points4D.at<float>(3,i) ) );
        cv::Point3d currentPoint;

        //currentPoint.x = needlePointsTmp[i].x / 1000;
        //currentPoint.y = needlePointsTmp[i].y / 1000;
        //currentPoint.z = needlePointsTmp[i].z / 1000;

        currentPoint.x = needlePointsTmp[i].x;
        currentPoint.y = needlePointsTmp[i].y;
        currentPoint.z = needlePointsTmp[i].z;

        needlePoints.push_back(currentPoint);
    }

    ofstream myfile;
    std::stringstream ss;
    std::string imageName;

    //ss << "Data/NeedlePoints.txt";

    imageName = ss.str();
    const char * imageNameChar = imageName.c_str();

    myfile.open(imageNameChar);

    for (unsigned int c=0; c < needlePoints.size(); c++)
        myfile << needlePoints[c].x << " " << needlePoints[c].y << " " << needlePoints[c].z << "\n";

    //myfile.close();
}

bool NeedlePoseEstimation::poseEstimation3D(  std::vector<cv::Point3d> &needle3Dmodel, cv::Mat &needleDriverPose, std::vector<cv::Point3d> &needlePoints, cv::Mat &finalNeedlePose )
{
    //reverse(needlePoints.begin(), needlePoints.end());

    //reverse(needle3Dmodel.begin(), needle3Dmodel.end());

    //double samplingDistance = 0.001;
    double samplingDistance = 1.0;

    bool continueLoop=true;
    double currentDistance = 0;
    int cont = 1;
    std::vector<cv::Point3d> needle3DModelSampled;
    needle3DModelSampled.push_back(needle3Dmodel[0]);
    while(continueLoop)
    {
        double distance = sqrt( pow(needle3Dmodel[cont-1].x-needle3Dmodel[cont].x,2) + pow(needle3Dmodel[cont-1].y-needle3Dmodel[cont].y,2) + pow(needle3Dmodel[cont-1].z-needle3Dmodel[cont].z,2) );
        currentDistance = currentDistance + distance;

        if(currentDistance >= samplingDistance)
        {
            needle3DModelSampled.push_back(needle3Dmodel[cont]);
            currentDistance = 0;
        }

        cont++;

        if(cont == needle3Dmodel.size())
            continueLoop = false;
    }

    continueLoop=true;
    currentDistance = 0;
    cont = 1;
    std::vector<cv::Point3d> needle3DReconstructedSampled;
    needle3DReconstructedSampled.push_back(needlePoints[0]);
    while(continueLoop)
    {
        double distance = sqrt( pow(needlePoints[cont-1].x-needlePoints[cont].x,2) + pow(needlePoints[cont-1].y-needlePoints[cont].y,2) + pow(needlePoints[cont-1].z-needlePoints[cont].z,2) );
        currentDistance = currentDistance + distance;

        if(currentDistance >= samplingDistance)
        {
            needle3DReconstructedSampled.push_back(needlePoints[cont]);
            currentDistance = 0;
        }

        cont++;

        if(cont == needlePoints.size())
            continueLoop = false;
    }

    if(needle3DReconstructedSampled.size() == needle3DModelSampled.size())
        ;
    else if(needle3DReconstructedSampled.size() < needle3DModelSampled.size())
        needle3DModelSampled.resize(needle3DReconstructedSampled.size());
    else
        needle3DReconstructedSampled.resize(needle3DModelSampled.size());

    pointMatch_rigid2(needle3DModelSampled, needle3DReconstructedSampled, finalNeedlePose);

    /*finalNeedlePose.at<double>(0,0)= 1.0; finalNeedlePose.at<double>(0,1)= 0.0; finalNeedlePose.at<double>(0,2)= 0.0; finalNeedlePose.at<double>(0,3)= 0.0;
    finalNeedlePose.at<double>(1,0)= 0.0; finalNeedlePose.at<double>(1,1)= 1.0; finalNeedlePose.at<double>(1,2)= 0.0; finalNeedlePose.at<double>(1,3)= 0.0;
    finalNeedlePose.at<double>(2,0)= 0.0; finalNeedlePose.at<double>(2,1)= 0.0; finalNeedlePose.at<double>(2,2)= 1.0; finalNeedlePose.at<double>(2,3)= 0.0;
    finalNeedlePose.at<double>(3,0)= 0.0; finalNeedlePose.at<double>(3,1)= 0.0; finalNeedlePose.at<double>(3,2)= 0.0; finalNeedlePose.at<double>(3,3)= 1.0;*/

    cout << finalNeedlePose <<endl;

    ofstream myfile1, myfile2, myfile3, myfile4, myfile5, myfile6;
    std::stringstream ss1, ss2, ss3, ss4, ss5, ss6;
    std::string imageName1, imageName2, imageName3, imageName4, imageName5, imageName6;

    ss1 << "Data/Model_sampled_camera.txt";
    ss2 << "Data/NeedleReconstruction_sampled_camera.txt";
    ss3 << "Data/Model_camera.txt";
    ss4 << "Data/newModel_camera.txt";
    ss5 << "Data/NeedleReconstruction_camera.txt";
    ss6 << "Data/finalNeedlePose_camera.txt";

    imageName1 = ss1.str();
    imageName2 = ss2.str();
    imageName3 = ss3.str();
    imageName4 = ss4.str();
    imageName5 = ss5.str();
    imageName6 = ss6.str();

    const char * imageNameChar1 = imageName1.c_str();
    const char * imageNameChar2 = imageName2.c_str();
    const char * imageNameChar3 = imageName3.c_str();
    const char * imageNameChar4 = imageName4.c_str();
    const char * imageNameChar5 = imageName5.c_str();
    const char * imageNameChar6 = imageName6.c_str();

    myfile1.open(imageNameChar1);
    myfile2.open(imageNameChar2);
    myfile3.open(imageNameChar3);
    myfile4.open(imageNameChar4);
    myfile5.open(imageNameChar5);
    myfile6.open(imageNameChar6);

    myfile6 << finalNeedlePose << endl;

    for (unsigned int c=0; c < needle3DModelSampled.size(); c++)
    {
        myfile1 << needle3DModelSampled[c].x << " " << needle3DModelSampled[c].y << " " << needle3DModelSampled[c].z << "\n";
        myfile2 << needle3DReconstructedSampled[c].x << " " << needle3DReconstructedSampled[c].y << " " << needle3DReconstructedSampled[c].z << "\n";
    }

    for (unsigned int c=0; c < needle3Dmodel.size(); c++)
        myfile3 << needle3Dmodel[c].x << " " << needle3Dmodel[c].y << " " << needle3Dmodel[c].z << "\n";

    Mat needle3DmodelPoints = Mat(4,needle3Dmodel.size(), CV_64F);
    Mat needle3DmodelPointsNew = Mat(4,needle3Dmodel.size(), CV_64F);

    for(int i=0; i<needle3Dmodel.size();i++)
    {
        needle3DmodelPoints.at<double>(0,i) = needle3Dmodel[i].x;
        needle3DmodelPoints.at<double>(1,i) = needle3Dmodel[i].y;
        needle3DmodelPoints.at<double>(2,i) = needle3Dmodel[i].z;
        needle3DmodelPoints.at<double>(3,i) = 1;
    }

    needle3DmodelPointsNew = finalNeedlePose * needle3DmodelPoints;

    std::vector<cv::Point3d> needle3DmodelNew;
    for(int i=0; i<needle3Dmodel.size();i++)
    {
        Point3d currentPoint;
        currentPoint.x = needle3DmodelPointsNew.at<double>(0,i);
        currentPoint.y = needle3DmodelPointsNew.at<double>(1,i);
        currentPoint.z = needle3DmodelPointsNew.at<double>(2,i);
        needle3DmodelNew.push_back(currentPoint);
    }

    for (unsigned int c=0; c < needle3DmodelNew.size(); c++)
        myfile4 << needle3DmodelNew[c].x << " " << needle3DmodelNew[c].y << " " << needle3DmodelNew[c].z << "\n";

    for (unsigned int c=0; c < needlePoints.size(); c++)
        myfile5 << needlePoints[c].x << " " << needlePoints[c].y << " " << needlePoints[c].z << "\n";

    myfile1.close();
    myfile2.close();
    myfile3.close();
    myfile4.close();
    myfile5.close();
    myfile6.close();
}

bool NeedlePoseEstimation::pointMatch_rigid2(std::vector<Point3d> &m_R, std::vector<Point3d> &m_C, cv::Mat &R_C)
{
    cout << "hrere"<<endl;
    find_rigidTransformation2(m_R,m_C,R_C);
    return true;
}

bool NeedlePoseEstimation::project3DModel( cv::Mat leftImage, cv::Mat rightImage, std::vector<cv::Point3d> &needle3Dmodel, std::vector<cv::Point2d> &needlePointsLeft_estimate, std::vector<cv::Point2d> &needlePointsRight_estimate)
{
    std::vector<cv::Point2d> needlePointsLeft;
    std::vector<cv::Point2d> needlePointsRight;

    cv::Mat P1, D1, P2, D2;

    FileStorage f_P1(CAMERA_CALIB_DIR + "P1.xml", FileStorage::READ); f_P1["P1"]>>P1;
    FileStorage f_D1(CAMERA_CALIB_DIR + "D1.xml", FileStorage::READ); f_D1["D1"]>>D1;
    FileStorage f_P2(CAMERA_CALIB_DIR + "P2.xml", FileStorage::READ); f_P2["P2"]>>P2;
    FileStorage f_D2(CAMERA_CALIB_DIR + "D2.xml", FileStorage::READ); f_D2["D2"]>>D2;

    Mat t_vec1 = Mat(3,1, CV_64F, double(0));
    Mat r_vec1 = Mat(3,1, CV_64F, double(0));

    Mat t_vec2 = Mat(3,1, CV_64F, double(0));
    t_vec2.at<double>(0,0)= P2.at<double>(0,3)/P2.at<double>(0,0);
    Mat r_vec2 = Mat(3,1, CV_64F, double(0));

    Mat P1_ = P1(cv::Rect(0,0,3,3));
    Mat P2_ = P2(cv::Rect(0,0,3,3));

    cv::projectPoints(needle3Dmodel, r_vec1, t_vec1, P1_, D1, needlePointsLeft);
    cv::projectPoints(needle3Dmodel, r_vec2, t_vec2, P2_, D2, needlePointsRight);

    for(int i=0; i<needlePointsLeft.size(); i++)
    {
     Point2d currentPoint;

     currentPoint.x = (int)(needlePointsLeft[i].x);
     currentPoint.y = (int)(needlePointsLeft[i].y);

     bool found = false;
     for(int j=0; j<needlePointsLeft_estimate.size(); j++)
     {
      Point2d currentPointCandidate = needlePointsLeft_estimate[j];
      if(currentPointCandidate == currentPoint)
          found = true;
     }

     if(!found)
        needlePointsLeft_estimate.push_back(currentPoint);
    }

    for(int i=0; i<needlePointsRight.size(); i++)
    {
     Point2d currentPoint;

     currentPoint.x = (int)(needlePointsRight[i].x);
     currentPoint.y = (int)(needlePointsRight[i].y);

     bool found = false;
     for(int j=0; j<needlePointsRight_estimate.size(); j++)
     {
      Point2d currentPointCandidate = needlePointsRight_estimate[j];
      if(currentPointCandidate == currentPoint)
          found = true;
     }

     if(!found)
        needlePointsRight_estimate.push_back(currentPoint);
    }

    /*for(int i=0; i<needlePointsLeft_estimate.size();i++)
    {
        circle(leftImage, needlePointsLeft_estimate[i], 2, Scalar(0,0,0), -1, 8, 0);
        circle(rightImage, needlePointsRight_estimate[i], 2, Scalar(0,0,0), -1, 8, 0);
    }

    imshow("frame left",leftImage);
    imshow("frame right",rightImage);
    cvWaitKey();*/
}

bool NeedlePoseEstimation::project3DModel2( cv::Mat leftImage, cv::Mat rightImage, std::vector<cv::Point3d> &needle3Dmodel, cv::Mat newNeedlePose, std::vector<cv::Point3d> &needlePoints)
{
    imwrite("Data/originalRect_right.png",rightImage);
    imwrite("Data/originalRect_left.png",leftImage);

    cv::Mat M1, D1, M2, D2, P1, P2;

    FileStorage f_P1(CAMERA_CALIB_DIR + "P1.xml", FileStorage::READ); f_P1["P1"]>>P1;
    FileStorage f_P2(CAMERA_CALIB_DIR + "P2.xml", FileStorage::READ); f_P2["P2"]>>P2;
    FileStorage f_D1(CAMERA_CALIB_DIR + "D1.xml", FileStorage::READ); f_D1["D1"]>>D1;
    FileStorage f_D2(CAMERA_CALIB_DIR + "D2.xml", FileStorage::READ); f_D2["D2"]>>D2;

    Mat t_vec1 = Mat(3,1, CV_64F, double(0));
    Mat r_vec1 = Mat(3,1, CV_64F, double(0));

    Mat t_vec2 = Mat(3,1, CV_64F, double(0));
    t_vec2.at<double>(0,0)= P2.at<double>(0,3)/P2.at<double>(0,0);
    Mat r_vec2 = Mat(3,1, CV_64F, double(0));

    Mat P1_ = P1(cv::Rect(0,0,3,3));
    Mat P2_ = P2(cv::Rect(0,0,3,3));

    Mat needle3DmodelPoints = Mat(4,needle3Dmodel.size(), CV_64F);
    Mat needle3DmodelPointsNew = Mat(4,needle3Dmodel.size(), CV_64F);

    for(int i=0; i<needle3Dmodel.size();i++)
    {
        needle3DmodelPoints.at<double>(0,i) = needle3Dmodel[i].x;
        needle3DmodelPoints.at<double>(1,i) = needle3Dmodel[i].y;
        needle3DmodelPoints.at<double>(2,i) = needle3Dmodel[i].z;
        needle3DmodelPoints.at<double>(3,i) = 1;
    }

    needle3DmodelPointsNew = newNeedlePose * needle3DmodelPoints;

    std::vector<cv::Point3d> needle3DmodelNew;
    for(int i=0; i<needle3Dmodel.size();i++)
    {
        Point3d currentPoint;
        currentPoint.x = needle3DmodelPointsNew.at<double>(0,i);
        currentPoint.y = needle3DmodelPointsNew.at<double>(1,i);
        currentPoint.z = needle3DmodelPointsNew.at<double>(2,i);
        needle3DmodelNew.push_back(currentPoint);
    }

    std::vector<cv::Point2d> needlePointsLeft;
    std::vector<cv::Point2d> needlePointsRight;

    cv::projectPoints(needle3DmodelNew, r_vec1, t_vec1, P1_, D1, needlePointsLeft);
    cv::projectPoints(needle3DmodelNew, r_vec2, t_vec2, P2_, D2, needlePointsRight);

    for(int i=0; i<needlePointsLeft.size();i++)
    {
        circle(leftImage, needlePointsLeft[i], 2, Scalar(0,255,0), -1, 8, 0);
        circle(rightImage, needlePointsRight[i], 2, Scalar(0,255,0), -1, 8, 0);
    }

    std::vector<cv::Point2d> needlePointsLeft_old;
    std::vector<cv::Point2d> needlePointsRight_old;

    cv::projectPoints(needle3Dmodel, r_vec1, t_vec1, P1_, D1, needlePointsLeft_old);
    cv::projectPoints(needle3Dmodel, r_vec2, t_vec2, P2_, D2, needlePointsRight_old);

    for(int i=0; i<needlePointsLeft_old.size();i++)
    {
        circle(leftImage, needlePointsLeft_old[i], 2, Scalar(0,0,255), -1, 8, 0);
        circle(rightImage, needlePointsRight_old[i], 2, Scalar(0,0,255), -1, 8, 0);
    }

    std::vector<cv::Point2d> needlePointsLeft_new;
    std::vector<cv::Point2d> needlePointsRight_new;

    cv::projectPoints(needlePoints, r_vec1, t_vec1, P1_, D1, needlePointsLeft_new);
    cv::projectPoints(needlePoints, r_vec2, t_vec2, P2_, D2, needlePointsRight_new);

    /*for(int i=0; i<needlePointsLeft_new.size();i++)
    {
        circle(leftImage, needlePointsLeft_new[i], 2, Scalar(255,255,255), -1, 8, 0);
        circle(rightImage, needlePointsRight_new[i], 2, Scalar(255,255,255), -1, 8, 0);
    }*/

    //imshow("frame left",leftImage);
    //imshow("frame right",rightImage);

    imwrite("Data/reconstructionRect_right.png",rightImage);
    imwrite("Data/reconstructionRect_left.png",leftImage);
    //cvWaitKey();
}
