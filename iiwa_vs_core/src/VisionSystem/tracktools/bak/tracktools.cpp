
#include "tracktools.h"



using namespace myaruco;
using namespace std;
using namespace cv;


/************************************
 *
 *
 *
 *
 ************************************/
template <typename CV_POINT3_SRC, typename CV_POINT3_DST>
static void find_rigidTransformation(const std::vector<CV_POINT3_SRC> &_src,
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
/************************************
 *
 *
 *
 *
 ************************************/



TrackTools::TrackTools(string fileDir)
{
    // Load stereo camera parameters
    FileStorage f_Q(fileDir + "Q.xml", FileStorage::READ); f_Q["Q"]>>Q;
    FileStorage f_mx1(fileDir + "mx1.xml", FileStorage::READ); f_mx1["mx1"]>>mx1;
    FileStorage f_my1(fileDir + "my1.xml", FileStorage::READ); f_my1["my1"]>>my1;
    FileStorage f_mx2(fileDir + "mx2.xml", FileStorage::READ); f_mx2["mx2"]>>mx2;
    FileStorage f_my2(fileDir + "my2.xml", FileStorage::READ); f_my2["my2"]>>my2;
    FileStorage f_P1(fileDir + "P1.xml", FileStorage::READ); f_P1["P1"]>>P1;
    FileStorage f_P2(fileDir + "P2.xml", FileStorage::READ); f_P2["P2"]>>P2;
    FileStorage f_D1(fileDir + "D1.xml", FileStorage::READ); f_P1["D1"]>>D1;
    FileStorage f_D2(fileDir + "D2.xml", FileStorage::READ); f_P2["D2"]>>D2;
    FileStorage f_T(fileDir + "T.xml", FileStorage::READ); f_T["T"]>>T;
    FileStorage f_R(fileDir + "R.xml", FileStorage::READ); f_R["R"]>>R;


    CameraL_ind = 0;
    theIntrinsicFileL= fileDir + "camera_left.yml";
    CameraR_ind = 1;
    theIntrinsicFileR = fileDir + "camera_right.yml";

    double ThresParam1 = 7, ThresParam2 = 7;

    //read camera parameters
    TheCameraParametersL.readFromXMLFile(theIntrinsicFileL);
    //TheCameraParametersL.resize(frameL.size());
    TheCameraParametersR.readFromXMLFile(theIntrinsicFileR);
    //TheCameraParametersR.resize(frameR.size());

    StereoCameraParametersL.readFromXMLFile(theIntrinsicFileL);
    //StereoCameraParametersL.resize(frameL.size());
    StereoCameraParametersR.readFromXMLFile(theIntrinsicFileR);
    //StereoCameraParametersR.resize(frameR.size());

    Mat P1_ = P1(cv::Rect(0,0,3,3));
    P1_.copyTo(StereoCameraParametersL.CameraMatrix);
    D1.copyTo(StereoCameraParametersL.Distorsion);

    //Configure other parameters
    if (ThePyrDownLevel>0)
        MDetector.pyrDown(ThePyrDownLevel);

    MDetectorL.getThresholdParams( ThresParam1,ThresParam2);
    MDetectorL.setCornerRefinementMethod(myaruco::MarkerDetector::LINES);
    MDetectorR.getThresholdParams( ThresParam1,ThresParam2);
    MDetectorR.setCornerRefinementMethod(myaruco::MarkerDetector::LINES);

}


void TrackTools::initTools(vector<float> markersSizes, vector<vector<int> > toolsids)
{
    numTools = markersSizes.size();
    for (int i=0; i<numTools; i++)
    {
        MarkersSize.push_back(markersSizes[i]);
        ToolsID.push_back(toolsids[i]);
    }
    Tools.resize(numTools);
    toolsDetected.resize(numTools);
}

void TrackTools::detectTools(cv::Mat3b frameL, cv::Mat3b frameR,
                             vector<cv::Mat> &toolsTvec, vector<cv::Mat> &toolsRvec, vector<bool> &detected)
{
    // Clean previous markers
    TheMarkersL.clear();
    TheMarkersR.clear();
    StereoMarkersL.clear();
    StereoMarkersR.clear();

    //Rectify image
    //remap(frameL, frameL, mx1, my1, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );
    //remap(frameR, frameR, mx2, my2, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );

    //Detection of markers in the image passed
    MDetectorL.detect(frameL,TheMarkersL,TheCameraParametersL,TheMarkerSize);
    MDetectorR.detect(frameR,TheMarkersR,TheCameraParametersR,TheMarkerSize);

    //Find markers showing in both cameras    
    findStereoMarkers();
    estimateStereoMarkerPose();

    for (int i=0; i<numTools; i++)
    {
//        Tool.clear();
//        Tool = Tools[i];
        toolsDetected[i] = estimateToolPose(Tools[i], ToolsID[i], ToolsID[i].size());
        Tools[i].Tvec.copyTo(toolsTvec[i]);
        Tools[i].Rvec.copyTo(toolsRvec[i]);

        detected.push_back(toolsDetected[i]);
    }

}


bool TrackTools::findStereoMarkers()
{
    // Search for markers that appear at both cameras
    int searchInd = 0;
    for (unsigned int i=0; i<TheMarkersL.size();i++)
    {
        for (unsigned int j=searchInd; j<TheMarkersR.size(); j++)
        {
            if (TheMarkersL[i].id == TheMarkersR[j].id)
            {
                StereoMarkersL.push_back(TheMarkersL[i]);
                StereoMarkersR.push_back(TheMarkersR[j]);
                searchInd = j+1;
                break;
            }

        }
    }

}

void TrackTools::estimateStereoMarkerPose()
{
    vector<Point3f> marker4corners;
    Point3f cornerIncam;
    Point2f cornerL, cornerR;
    Mat point4D;
    Mat markerInCameraTrans;

    Mat P1_ = P1(cv::Rect(0,0,3,3));

    Mat t_vec1 = Mat(3,1, CV_64F, double(0));
    Mat r_vec1 = Mat(3,1, CV_64F, double(0));

    for (unsigned int i=0; i<StereoMarkersL.size(); i++)
    {
        marker4corners.clear();
        for (int j=0; j<4; j++)
        {
            // Computer the corners' 3D location in sereo camera frame
            cornerL.x = StereoMarkersL[i][j].x;
            cornerL.y = StereoMarkersL[i][j].y;
            cornerR.x = StereoMarkersR[i][j].x;
            cornerR.y = StereoMarkersR[i][j].y;

            triangulatePoints(P1, P2, Mat(cornerL), Mat(cornerR), point4D);

            cornerIncam.x = point4D.at<float>(0,0)/point4D.at<float>(0,3);
            cornerIncam.y = point4D.at<float>(0,1)/point4D.at<float>(0,3);
            cornerIncam.z = point4D.at<float>(0,2)/point4D.at<float>(0,3);

//            cout<< "cornerIncam: " << cornerIncam << endl;
            marker4corners.push_back(cornerIncam);

        }

        // Reproject 3D points to left image
        vector<Point2f> imagePnt;
        cv::projectPoints(marker4corners, r_vec1, t_vec1, P1_, D1, imagePnt);
        float size_ = 0;
        float temp_x = 0, temp_y = 0, temp_z = 0;
        for (int j=0; j<4; j++)
        {
            StereoMarkersL[i][j].x = imagePnt[j].x;
            StereoMarkersL[i][j].y = imagePnt[j].y;
            size_ = size_ + norm(Mat(marker4corners[j%4]), Mat(marker4corners[(j+1)%4]));
            temp_x += marker4corners[j].x/4;
            temp_y += marker4corners[j].y/4;
            temp_z += marker4corners[j].z/4;
        }


        StereoMarkersL[i].ssize = size_/4;
        double halfSize=StereoMarkersL[i].ssize/2.;
        vector<Point3f> markercornerLocal;
        markercornerLocal.resize(4);
           markercornerLocal[0].x = -halfSize;
           markercornerLocal[0].y = -halfSize;
           markercornerLocal[0].z = 0;
           markercornerLocal[1].x = -halfSize;
           markercornerLocal[1].y = halfSize;
           markercornerLocal[1].z = 0;
           markercornerLocal[2].x = halfSize;
           markercornerLocal[2].y = halfSize;
           markercornerLocal[2].z = 0;
           markercornerLocal[3].x = halfSize;
           markercornerLocal[3].y = -halfSize;
           markercornerLocal[3].z = 0;

        find_rigidTransformation(markercornerLocal,marker4corners,markerInCameraTrans);

        // Why need to rotate ????????--------------------------
        Mat tranX90 = (Mat_<double>(4,4) << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1);
        markerInCameraTrans = markerInCameraTrans * tranX90;


        StereoMarkersL[i].Tvec.ptr<float>(0)[0] = markerInCameraTrans.at<double>(0,3);
        StereoMarkersL[i].Tvec.ptr<float>(0)[1] = markerInCameraTrans.at<double>(1,3);
        StereoMarkersL[i].Tvec.ptr<float>(0)[2] = markerInCameraTrans.at<double>(2,3);

        Mat rotationMat = markerInCameraTrans.colRange(0, 3).rowRange(0, 3);

        // Correct pose from left
        cv::Mat rvec;
        cv::Rodrigues(rotationMat, rvec);

        StereoMarkersL[i].Tvec.convertTo(StereoMarkersL[i].Tvec, CV_32F);
        rvec.convertTo(StereoMarkersL[i].Rvec, CV_32F);

    }


}

bool TrackTools::estimateToolPose(aruco::Marker &tool, vector<int> toolID, int N)
{
    bool toolDetected = false;

    Mat markerInCameraTrans = Mat::eye(4,4,CV_32F),
        toolInMarkerTrans= Mat::eye(4,4,CV_32F),
        toolInCameraTrans = Mat::eye(4,4,CV_32F);

    //int N = (sizeof(toolID)/sizeof(toolID[2]));
    double rotXangle;
    int ind;
    for (int i=0; i<N; i++)
    {
        for (int j=0; j<StereoMarkersL.size(); j++)
        {
            if (StereoMarkersL[j].id == toolID[i])
            {
                ind = j;
                rotXangle = 2*M_PI / N * i;
                toolDetected = true;
                break;
            }
        }
    }

    if (toolDetected)
    {
        markerInCameraTrans.at<float>(0,3) = StereoMarkersL[ind].Tvec.ptr<float>(0)[0];
        markerInCameraTrans.at<float>(1,3) = StereoMarkersL[ind].Tvec.ptr<float>(0)[1];
        markerInCameraTrans.at<float>(2,3) = StereoMarkersL[ind].Tvec.ptr<float>(0)[2];

        cv::Rodrigues(StereoMarkersL[ind].Rvec, markerInCameraTrans.colRange(0,3).rowRange(0,3));

        toolInMarkerTrans.at<float>(1,1) = cos(rotXangle); toolInMarkerTrans.at<float>(1,2) = -sin(rotXangle);
        toolInMarkerTrans.at<float>(2,1) = sin(rotXangle); toolInMarkerTrans.at<float>(2,2) =  cos(rotXangle);
        // Shift along y axis
        toolInMarkerTrans.at<float>(1,3) =  - StereoMarkersL[ind].ssize/(2*tan(M_PI/N));

        // Compute tool pose
        toolInCameraTrans = markerInCameraTrans * toolInMarkerTrans;


        tool.Tvec.ptr<float>(0)[0] = toolInCameraTrans.at<float>(0,3);
        tool.Tvec.ptr<float>(0)[1] = toolInCameraTrans.at<float>(1,3);
        tool.Tvec.ptr<float>(0)[2] = toolInCameraTrans.at<float>(2,3);

        // Correct pose for left
        cv::Mat rvec;
        cv::Rodrigues(toolInCameraTrans.colRange(0,3).rowRange(0,3), rvec);

        tool.Tvec.convertTo(tool.Tvec, CV_32F);
        rvec.convertTo(tool.Rvec, CV_32F);
        tool.ssize = StereoMarkersL[ind].ssize;

  //      cout << "Tool position: " << Tool.Tvec.ptr<float>(0)[0] << " "
    //         << Tool.Tvec.ptr<float>(0)[1] << " " << Tool.Tvec.ptr<float>(0)[2] << endl;
        return true;
    }
    else
    {
        tool.Tvec.ptr<float>(0)[0] = -1000;
        tool.Tvec.ptr<float>(0)[1] = -1000;
        tool.Tvec.ptr<float>(0)[2] = -1000;
        tool.Rvec.ptr<float>(0)[0] = -1000;
        tool.Rvec.ptr<float>(0)[1] = -1000;
        tool.Rvec.ptr<float>(0)[2] = -1000;
        tool.ssize = -1000;
        return false;
    }

}


void TrackTools::drawStereoMarkers(cv::Mat3b frameL, cv::Mat3b frameR)
{
    // draw markers detected in the right frame
    for (unsigned int i=0;i<TheMarkersR.size();i++)
    {
        TheMarkersR[i].draw(frameR,Scalar(0,0,255),1);
    }

    //draw markers in the left frame if there is 3d info
    for (unsigned int i=0;i<StereoMarkersL.size();i++)
    {
        //CvDrawingUtils::draw3dCube(frameL,StereoMarkersL[i],StereoCameraParametersL);
        //myCvDrawingUtils::draw3dAxis(frameL,StereoMarkersL[i],StereoCameraParametersL);
        StereoMarkersL[i].draw(frameL,Scalar(0,255,0),1);
    }
}

void TrackTools::drawTools(cv::Mat3b frameL)
{
    // Only draw on the left frame!
    for (int i=0; i<numTools; i++)
    {
        if (toolsDetected[i])
        {
            CvDrawingUtils::draw3dAxis(frameL, Tools[i], StereoCameraParametersL);
        }
    }
}

void TrackTools::drawTools_smooth(cv::Mat3b frameL, vector<cv::Mat> &toolTvec_smooth, vector<cv::Mat> &toolRvec_smooth)
{
    vector<aruco::Marker> Tools_smooth;

    for (int i=0; i<numTools; i++)
    {
        Tools_smooth.push_back(Tools[i]);
        toolTvec_smooth[i].copyTo(Tools_smooth[i].Tvec);
        toolRvec_smooth[i].copyTo(Tools_smooth[i].Rvec);

        if (Tools_smooth[i].Tvec.ptr<float>(0)[0] != 0 &&
            Tools_smooth[i].Tvec.ptr<float>(0)[1] != 0 &&
            Tools_smooth[i].Tvec.ptr<float>(0)[2] != 0) // if i-th tool has been seen
            CvDrawingUtils::draw3dAxis(frameL, Tools_smooth[i], StereoCameraParametersL);
    }
}

void TrackTools::toolPose2cvTrans(Marker toolp, cv::Mat &trans)
{
    trans = Mat::eye(4,4,CV_32F);

    trans.at<float>(0,3) = toolp.Tvec.ptr<float>(0)[0];
    trans.at<float>(1,3) = toolp.Tvec.ptr<float>(0)[1];
    trans.at<float>(2,3) = toolp.Tvec.ptr<float>(0)[2];

    cv::Rodrigues(toolp.Rvec, trans.colRange(0,3).rowRange(0,3));
}

void TrackTools::cvTrans2toolPose(cv::Mat trans, Marker &toolp)
{

    toolp.Tvec.ptr<float>(0)[0] = trans.at<float>(0,3);
    toolp.Tvec.ptr<float>(0)[1] = trans.at<float>(1,3);
    toolp.Tvec.ptr<float>(0)[2] = trans.at<float>(2,3);

    cv::Mat rvec;
    cv::Rodrigues(trans.colRange(0,3).rowRange(0,3), rvec);

    toolp.Tvec.convertTo(toolp.Tvec, CV_32F);
    rvec.convertTo(toolp.Rvec, CV_32F);
}
