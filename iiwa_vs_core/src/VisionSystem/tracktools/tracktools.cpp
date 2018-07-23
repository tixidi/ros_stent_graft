
#include "tracktools.h"



using namespace myaruco;
using namespace std;
using namespace cv;
using namespace dvrk;

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


void TrackTools::initTools(vector<float> markersSizes, vector<vector<int> > toolsids, int fWindow)
{
    numTools = markersSizes.size();
    for (int i=0; i<numTools; i++)
    {
        MarkersSize.push_back(markersSizes[i]);
        ToolsID.push_back(toolsids[i]);
    }
    Tools.resize(numTools);
    toolsDetected.resize(numTools);

    this->filterwindow = fWindow;
    if (filterwindow > 0)
    {
        Tools_hist.resize(numTools);
        fcount = 0;
    }
    initTrackers();

    // Kalman filter ---------------------------------
    // initialization of kf --------------------------
    KFtool.resize(numTools);
    KFtool_rot.resize(numTools);
    Tools_previous.resize(numTools);
    Tools_kalman.resize(numTools);

//    for (int i=0; i<numTools; i++)
//    {
//        fristDetected.push_back(true); //first time detected tool
//        KFtool[i] = KalmanFilter(6,3,0);
//        KFtool_rot[i] = KalmanFilter(6,3,0);
//        initializeKF(KFtool[i], KFtool_rot[i]);
//    }
    // ===============================================
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
        toolsDetected[i] = estimateToolPose(Tools[i], ToolsID[i], ToolsID[i].size(), MarkersSize[i]);
        //toolsDetected[i] = estimateToolPose_multimarkers(Tools[i], ToolsID[i], ToolsID[i].size(), MarkersSize[i]);
        Tools[i].Tvec.copyTo(toolsTvec[i]);
        Tools[i].Rvec.copyTo(toolsRvec[i]);

        detected.push_back(toolsDetected[i]);
    }


    ////////////////// Moving average //////////////////
    if (filterwindow > 0)
    {
        for (int i=0; i<numTools; i++)
        { //i-th tool
            if (toolsDetected[i])
            {
                // store history
                if (Tools_hist[i].size()<filterwindow)
                {
                    Tools_hist[i].push_back(Tools[i]);
                }
                else
                {
                    Tools[i].Tvec.copyTo(Tools_hist[i][fcount].Tvec);
                    Tools[i].Rvec.copyTo(Tools_hist[i][fcount].Rvec);
                    Tools_hist[i][fcount].ssize = Tools[i].ssize;
                }
            }
        }

        // Compute average
        for (int i=0; i<numTools; i++)
        {//i-th tool
            vector<float> sum_Tvec, sum_Rvec;
            sum_Tvec.push_back(0); sum_Tvec.push_back(0); sum_Tvec.push_back(0);
            sum_Rvec.push_back(0); sum_Rvec.push_back(0); sum_Rvec.push_back(0);

            for (int k=0; k<3; k++)
            {// k-th dimention in Tvec Rvec
                for (int j=0; j<Tools_hist[i].size(); j++)
                {// j-th history data
                    if (Tools_hist[i][j].Tvec.ptr<float>(0)[k] != -1000)
                    {
                        sum_Tvec[k] = sum_Tvec[k] + Tools_hist[i][j].Tvec.ptr<float>(0)[k];
                        sum_Rvec[k] = sum_Rvec[k] + Tools_hist[i][j].Rvec.ptr<float>(0)[k];
                    }
                }
                Tools[i].Tvec.ptr<float>(0)[k] = sum_Tvec[k]/Tools_hist[i].size();
                //Tools[i].Rvec.ptr<float>(0)[k] = sum_Rvec[k]/Tools_hist[i].size();
            }
        }

        // reset counter
        fcount++;
        if (fcount > filterwindow-1)
            fcount = 0;
    ///////////////////////////////////////////////////////////////

    }

}


void TrackTools::trackTools(cv::Mat3b frameL, cv::Mat3b frameR,
                             vector<cv::Mat> &toolsTvec, vector<cv::Mat> &toolsRvec, vector<bool> &detected)
{
    // Clean previous markers
    TheMarkersL.clear();
    TheMarkersR.clear();
    StereoMarkersL.clear();
    StereoMarkersR.clear();

    //Detection of markers in the image passed
    MDetectorL.detect(frameL,TheMarkersL,TheCameraParametersL,TheMarkerSize);
    MDetectorR.detect(frameR,TheMarkersR,TheCameraParametersR,TheMarkerSize);

    //Track Markers -------------------
    vector<bool> trakerMarksL_bool = trackMarkers(frameL, TheMarkersL,
                                                   MarkersL_track, trackersL,
                                                   TheCameraParametersL,TheMarkerSize);
    vector<bool> trakerMarksR_bool = trackMarkers(frameR, TheMarkersR,
                                                   MarkersR_track, trackersR,
                                                   TheCameraParametersR,TheMarkerSize);

    //Find markers showing in both cameras
    findStereoMarkers();
    //findStereoMarkers_tracker(trakerMarksL_bool, trakerMarksR_bool);
//    filterStereoMarkers(trakerMarksL_bool, MarkersL_track, MarkersL_hist, TheMarkersL,
//                        TheCameraParametersL,TheMarkerSize);
//    filterStereoMarkers(trakerMarksR_bool, MarkersR_track, MarkersR_hist, TheMarkersR,
//                        TheCameraParametersR,TheMarkerSize);
    estimateStereoMarkerPose();

    for (int i=0; i<numTools; i++)
    {
        if (ToolsID[i].size() == 8) //Mandrel
            toolsDetected[i] = estimateToolPose(Tools[i], ToolsID[i], ToolsID[i].size(), MarkersSize[i]);
        else
            if (ToolsID[i].size() == 1) //Suture device
                toolsDetected[i] = estimateToolPose_singlemarker(Tools[i], ToolsID[i], ToolsID[i].size(), MarkersSize[i]);
            else
                toolsDetected[i] = estimateToolPose_multimarkers(Tools[i], ToolsID[i], ToolsID[i].size(), MarkersSize[i]);

        Tools[i].Tvec.copyTo(toolsTvec[i]);
        Tools[i].Rvec.copyTo(toolsRvec[i]);

        detected.push_back(toolsDetected[i]);

//        // Kalman filter---------------------------------------
//        if (fristDetected[i])
//        {
//            Tools[i].Tvec.copyTo(Tools_previous[i].Tvec);
//            Tools[i].Rvec.copyTo(Tools_previous[i].Rvec);
//        }

//        kalmanFilterPose(Tools[i], Tools_previous[i],
//                               Tools_kalman[i], KFtool[i],
//                               KFtool_rot[i], i);

//        Tools_kalman[i].Tvec.copyTo(Tools_previous[i].Tvec);
//        Tools_kalman[i].Rvec.copyTo(Tools_previous[i].Rvec);
//        Tools_previous[i].ssize = Tools[i].ssize;
    }

//    //Bidan hardcode for validation--------------
#ifdef SimulationON
//    char *fname_tools = "../source/trajectories/result1.txt";
//    char *fname_tools = "../source/trajectories/test_5L_dri_inv";
    char *fname_tools = strdup((SRC_FILES_DIR+"trajectories/toolmandrel_2018-03-22-15-18.txt_inserted_smooth_quat").c_str());
    strdup((SRC_FILES_DIR+"trajectories/toolmandrel_2017-07-19-21-18.txt_smooth_driver_inv").c_str());
    ifstream f_stream(fname_tools);
    f_stream >> Tools[0].Tvec.ptr<float>(0)[0];
    f_stream >> Tools[0].Tvec.ptr<float>(0)[1];
    f_stream >> Tools[0].Tvec.ptr<float>(0)[2];
    f_stream >> Tools[0].Rvec.ptr<float>(0)[0];
    f_stream >> Tools[0].Rvec.ptr<float>(0)[1];
    f_stream >> Tools[0].Rvec.ptr<float>(0)[2];

//    Tools[0].Tvec.ptr<float>(0)[0] = 0.0527306;
//    Tools[0].Tvec.ptr<float>(0)[1] = 0.0420413;
//    Tools[0].Tvec.ptr<float>(0)[2] = 0.171104;
//    Tools[0].Rvec.ptr<float>(0)[0] = 0.292995;
//    Tools[0].Rvec.ptr<float>(0)[1] = -1.18839;
//    Tools[0].Rvec.ptr<float>(0)[2] = -1.18305;

    Tools[0].id = 321;
    Tools[0].ssize = 0.00853492; //Tools[1].ssize;
    toolsDetected[0] = true;
    detected[0] = true;
#endif
////    // ============================



//////        //Bidan hardcode for continous experiment--------------
//        Tools[0].Tvec.ptr<float>(0)[0] = 0.0524908;
//        Tools[0].Tvec.ptr<float>(0)[1] = 0.0420313;
//        Tools[0].Tvec.ptr<float>(0)[2] = 0.170587;
//        Tools[0].Rvec.ptr<float>(0)[0] = 0.906668;
//        Tools[0].Rvec.ptr<float>(0)[1] = -0.930777;
//        Tools[0].Rvec.ptr<float>(0)[2] = -1.39381;
//        Tools[0].id = 97;
//        Tools[0].ssize = 0.0194939;
//        toolsDetected[0] = true;
//        detected[0] = true;
//        // ============================

//    cout << "man " << Tools[0].Tvec.ptr<float>(0)[0] << " "
//                   << Tools[0].Tvec.ptr<float>(0)[1] << " "
//                   << Tools[0].Tvec.ptr<float>(0)[2] << " "
//                   << Tools[0].Rvec.ptr<float>(0)[0] << " "
//                    << Tools[0].Rvec.ptr<float>(0)[1] << " "
//                    << Tools[0].Rvec.ptr<float>(0)[2] << " "
//                    <<Tools[0].ssize                                  << endl;

    ////////////////// Moving average //////////////////
    if (filterwindow > 0)
    {
        for (int i=0; i<numTools; i++)
        { //i-th tool
            if (toolsDetected[i])
            {
                // store history
                if (Tools_hist[i].size()<filterwindow)
                {
                    Tools_hist[i].push_back(Tools[i]);
                }
                else
                {
                    Tools[i].Tvec.copyTo(Tools_hist[i][fcount].Tvec);
                    Tools[i].Rvec.copyTo(Tools_hist[i][fcount].Rvec);
                    Tools_hist[i][fcount].ssize = Tools[i].ssize;
                }
            }
        }

        // Compute average
        for (int i=0; i<numTools; i++)
        {//i-th tool
            vector<float> sum_Tvec, sum_Rvec;
            sum_Tvec.push_back(0); sum_Tvec.push_back(0); sum_Tvec.push_back(0);
            sum_Rvec.push_back(0); sum_Rvec.push_back(0); sum_Rvec.push_back(0);

            for (int k=0; k<3; k++)
            {// k-th dimention in Tvec Rvec
                for (int j=0; j<Tools_hist[i].size(); j++)
                {// j-th history data
                    if (Tools_hist[i][j].Tvec.ptr<float>(0)[k] != -1000)
                    {
                        sum_Tvec[k] = sum_Tvec[k] + Tools_hist[i][j].Tvec.ptr<float>(0)[k];
                        sum_Rvec[k] = sum_Rvec[k] + Tools_hist[i][j].Rvec.ptr<float>(0)[k];
                    }
                }

//                if (dualArmPlanner->getRobotInstance(0)->getQuaternion().dot(
//                            EEInRobot_0[0].getQuaternion()) < 0)
//                {
//                    EEInRobot_0[0].getQuaternion().w() *= -1;
//                    EEInRobot_0[0].getQuaternion().x() *= -1;
//                    EEInRobot_0[0].getQuaternion().y() *= -1;
//                    EEInRobot_0[0].getQuaternion().z() *= -1;
//                }
                //Tools[i].Tvec.ptr<float>(0)[k] = sum_Tvec[k]/Tools_hist[i].size();
                //Tools[i].Rvec.ptr<float>(0)[k] = sum_Rvec[k]/Tools_hist[i].size();
            }
        }

        // reset counter
        fcount++;
        if (fcount > filterwindow-1)
            fcount = 0;
    ///////////////////////////////////////////////////////////////

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
                std::cout << TheMarkersL[i].id << std::endl;
            }

        }
    }

}

void TrackTools::findStereoMarkers_tracker(vector<bool> markersL_bool, vector<bool> markersR_bool)
{
    for (unsigned int i=0; i<markersL_bool.size();i++)
    {
        if (markersL_bool[i] && markersR_bool[i])
        {// if marker is detected or tracked in both image
            StereoMarkersL.push_back(MarkersL_track[i]);
            StereoMarkersR.push_back(MarkersR_track[i]);
        }
    }
}

void TrackTools::filterStereoMarkers(vector<bool> markers_bool,
                                     vector<Marker> &Markers_track,
                                     vector<vector<Marker> > &Markers_hist,
                                     vector<Marker> &Markers_detect,
                                     CameraParameters camParams ,float markerSizeMeters)
{
    for (unsigned int i=0; i<markers_bool.size();i++)
    {// i-th marker
        if (markers_bool[i])
        {// if marker is detected or tracked in both image
            if (Markers_hist[i].size() == filterwindow)
            {// if history is full, remove the oldest one
                //Markers_hist[i].earse(Markers_hist[i].begin());
                Markers_hist[i].erase(Markers_hist[i].begin());
            }

            Markers_hist[i].push_back(Markers_track[i]);
            // filter the stereo markers and the marker tracker
            for (int z=0; z<4; z++)
            {// z-th corner
                float sumX=0; float sumY = 0;
                for (int k=0; k<Markers_hist[i].size(); k++)
                {// sum over history, k-th record
                    sumX = sumX + Markers_hist[i][k][z].x;
                    sumY = sumY + Markers_hist[i][k][z].y;
                }
                Markers_track[i][z].x = sumX/Markers_hist[i].size();
                Markers_track[i][z].y = sumY/Markers_hist[i].size();
                Markers_track[i].calculateExtrinsics(markerSizeMeters,camParams);
            }
            for (int l=0; l<Markers_detect.size(); l++)
            {
                if (Markers_track[i].id == Markers_detect[l].id)
                {
                    for (int m=0; m<4; m++)
                    {
                        Markers_detect[l][m].x = Markers_track[i][m].x;
                        Markers_detect[l][m].y = Markers_track[i][m].y;
                    }
                }
                Markers_detect[l].calculateExtrinsics(markerSizeMeters,camParams);
            }
        }
        else
        {
            if (Markers_hist[i].size()>0)
                Markers_hist[i].clear();
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

bool TrackTools::estimateToolPose(aruco::Marker &tool, vector<int> toolID, int N, float markersize)
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
                tool.id = toolID[i];
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
//        toolInMarkerTrans.at<float>(1,3) =  - StereoMarkersL[ind].ssize/(2*tan(M_PI/N));
        toolInMarkerTrans.at<float>(1,3) =  - markersize/(2*tan(M_PI/N));

//        if (N==8)
//        {//Mandrel hard code
//            toolInMarkerTrans.at<float>(1,3) =  - 0.066/2;
//        }

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
        //////// Use tracking ////////


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


bool TrackTools::estimateToolPose_multimarkers(aruco::Marker &tool, vector<int> toolID, int N, float markersize)
{
    bool toolDetected = false;

    Mat markerInCameraTrans = Mat::eye(4,4,CV_32F),
        toolInMarkerTrans= Mat::eye(4,4,CV_32F),
        toolInCameraTrans = Mat::eye(4,4,CV_32F);

    //int N = (sizeof(toolID)/sizeof(toolID[2]));
    double rotXangle[2];
    vector<int> markerInd(2);
    int detectedTimes=0;
    Marker tool_est;
    vector<Marker> tool_all;

    for (int i=0; i<N; i++)
    {
        for (int j=0; j<StereoMarkersL.size(); j++)
        {
            if (StereoMarkersL[j].id == toolID[i])
            {
                detectedTimes += 1;
                tool.id = toolID[i];
                markerInd[detectedTimes-1] = j;
                rotXangle[detectedTimes-1] = 2*M_PI / N * i;
                toolDetected = true;
                if (detectedTimes == 2)
                    break;
            }
        }
        if (detectedTimes == 2)
            break;
    }

    if (toolDetected)
    {
        for (int i=0; i<detectedTimes; i++)
        {
            markerInCameraTrans.at<float>(0,3) = StereoMarkersL[markerInd[i]].Tvec.ptr<float>(0)[0];
            markerInCameraTrans.at<float>(1,3) = StereoMarkersL[markerInd[i]].Tvec.ptr<float>(0)[1];
            markerInCameraTrans.at<float>(2,3) = StereoMarkersL[markerInd[i]].Tvec.ptr<float>(0)[2];

            cv::Rodrigues(StereoMarkersL[markerInd[i]].Rvec, markerInCameraTrans.colRange(0,3).rowRange(0,3));

            toolInMarkerTrans.at<float>(1,1) = cos(rotXangle[i]); toolInMarkerTrans.at<float>(1,2) = -sin(rotXangle[i]);
            toolInMarkerTrans.at<float>(2,1) = sin(rotXangle[i]); toolInMarkerTrans.at<float>(2,2) =  cos(rotXangle[i]);

            // Shift along y axis
    //        toolInMarkerTrans.at<float>(1,3) =  - StereoMarkersL[markerInd].ssize/(2*tan(M_PI/N));
            toolInMarkerTrans.at<float>(1,3) =  - markersize/(2*tan(M_PI/N));

            // Compute tool pose
            toolInCameraTrans = markerInCameraTrans * toolInMarkerTrans;
            // Correct pose for left
            cv::Mat rvec;
            cv::Rodrigues(toolInCameraTrans.colRange(0,3).rowRange(0,3), rvec);

            // Assign tool ----------------
            tool_est.Tvec.ptr<float>(0)[0] = toolInCameraTrans.at<float>(0,3);
            tool_est.Tvec.ptr<float>(0)[1] = toolInCameraTrans.at<float>(1,3);
            tool_est.Tvec.ptr<float>(0)[2] = toolInCameraTrans.at<float>(2,3);

            tool_est.Tvec.convertTo(tool_est.Tvec, CV_32F);
            rvec.convertTo(tool_est.Rvec, CV_32F);
            tool_est.ssize = StereoMarkersL[markerInd[i]].ssize;
            tool_est.id = StereoMarkersL[markerInd[i]].id;

//            if (N==5)
//            {
//                cout << tool_est.id
//                     << " " << tool_est.Tvec.ptr<float>(0)[0]
//                     << " " << tool_est.Tvec.ptr<float>(0)[1]
//                     << " " << tool_est.Tvec.ptr<float>(0)[2]
//                     << " " << tool_est.Rvec.ptr<float>(0)[0]
//                     << " " << tool_est.Rvec.ptr<float>(0)[1]
//                     << " " << tool_est.Rvec.ptr<float>(0)[2]
//                     << endl;
//            }

            tool_all.push_back(tool_est);
        }

        for (int i=1; i<detectedTimes; i++)
        {
            if (tool_all[0].Rvec.ptr<float>(0)[0] * tool_all[i].Rvec.ptr<float>(0)[0] +
                tool_all[0].Rvec.ptr<float>(0)[1] * tool_all[i].Rvec.ptr<float>(0)[1] +
                tool_all[0].Rvec.ptr<float>(0)[2] * tool_all[i].Rvec.ptr<float>(0)[2] < 0)
            {
                tool_all[i].Rvec.ptr<float>(0)[0] *= -1;
                tool_all[i].Rvec.ptr<float>(0)[1] *= -1;
                tool_all[i].Rvec.ptr<float>(0)[2] *= -1;
//                cout << "Reverse tool_est!" << endl;
            }
        }

        if (detectedTimes >1)
        {
            // Compute markers' weight ----------------
            float weight[detectedTimes];
            float sumWeight=0;

            for (int i=0; i<detectedTimes; i++)
            {
                weight[i] = pow(markerAera(markerInd[i]),4);
                sumWeight += weight[i];
            }

            if (sumWeight >0)
            {// If areas are computed correctly

                tool_est.Tvec.ptr<float>(0)[0] = 0;
                tool_est.Tvec.ptr<float>(0)[1] = 0;
                tool_est.Tvec.ptr<float>(0)[2] = 0;
                tool_est.Rvec.ptr<float>(0)[0] = 0;
                tool_est.Rvec.ptr<float>(0)[1] = 0;
                tool_est.Rvec.ptr<float>(0)[2] = 0;

                for (int i=0; i<detectedTimes; i++)
                {// TODO: handle rotation!!
                    tool_est.Tvec.ptr<float>(0)[0] += tool_all[i].Tvec.ptr<float>(0)[0] * weight[i]/sumWeight;
                    tool_est.Tvec.ptr<float>(0)[1] += tool_all[i].Tvec.ptr<float>(0)[1] * weight[i]/sumWeight;
                    tool_est.Tvec.ptr<float>(0)[2] += tool_all[i].Tvec.ptr<float>(0)[2] * weight[i]/sumWeight;
                    tool_est.Rvec.ptr<float>(0)[0] += tool_all[i].Rvec.ptr<float>(0)[0] * weight[i]/sumWeight;
                    tool_est.Rvec.ptr<float>(0)[1] += tool_all[i].Rvec.ptr<float>(0)[1] * weight[i]/sumWeight;
                    tool_est.Rvec.ptr<float>(0)[2] += tool_all[i].Rvec.ptr<float>(0)[2] * weight[i]/sumWeight;

//                    if (N==5)
//                    {
//                        cout << i << " " << tool_all[i].id << " " << tool_all[i].Tvec.ptr<float>(0)[0] <<
//                                " " << weight[i] << " " << sumWeight << " ";
////                    cout << " Marker " << StereoMarkersL[markerInd[i]].id << ": " << weight[i] << " " ;
////                        cout << "Tool " << tool_est.id
//                    }
                }
//                cout << endl;
            }
          }


        for (int i=0; i<3; i++)
        {
            tool.Tvec.ptr<float>(0)[i] = tool_est.Tvec.ptr<float>(0)[i];
            tool.Rvec.ptr<float>(0)[i] = tool_est.Rvec.ptr<float>(0)[i];
        }
        tool.ssize = tool_est.ssize;

//        cout << "Tool " << tool.id
//             << " " << tool.Tvec.ptr<float>(0)[0]
//             << " " << tool.Tvec.ptr<float>(0)[1]
//             << " " << tool.Tvec.ptr<float>(0)[2]
//             << " " << tool.Rvec.ptr<float>(0)[0]
//             << " " << tool.Rvec.ptr<float>(0)[1]
//             << " " << tool.Rvec.ptr<float>(0)[2]
//             << " " << tool.ssize
//             << endl;

        return true;

    }

    //////// Use tracking ////////
    tool.Tvec.ptr<float>(0)[0] = -1000;
    tool.Tvec.ptr<float>(0)[1] = -1000;
    tool.Tvec.ptr<float>(0)[2] = -1000;
    tool.Rvec.ptr<float>(0)[0] = -1000;
    tool.Rvec.ptr<float>(0)[1] = -1000;
    tool.Rvec.ptr<float>(0)[2] = -1000;
    tool.ssize = -1000;
    return false;

}

bool TrackTools::estimateToolPose_singlemarker(aruco::Marker &tool, vector<int> toolID, int N, float markersize)
{
    bool toolDetected = false;
    vector<int> markerInd(2);
    Marker tool_est;
    vector<Marker> tool_all;

    for (int i=0; i<N; i++)
    {
        for (int j=0; j<StereoMarkersL.size(); j++)
        {
            if (StereoMarkersL[j].id == toolID[i])
            {
                tool.id = toolID[i];
                markerInd[0] = j;
                toolDetected = true;
                break;
            }
        }
    }

    if (toolDetected)
    {
        for (int i=0; i<3; i++)
        {
            tool.Tvec.ptr<float>(0)[i] = StereoMarkersL[markerInd[0]].Tvec.ptr<float>(0)[i];
            tool.Rvec.ptr<float>(0)[i] = StereoMarkersL[markerInd[0]].Rvec.ptr<float>(0)[i];
        }
        tool.ssize = StereoMarkersL[markerInd[0]].ssize;
        return true;
    }

    //////// Use tracking ////////
    tool.Tvec.ptr<float>(0)[0] = -1000;
    tool.Tvec.ptr<float>(0)[1] = -1000;
    tool.Tvec.ptr<float>(0)[2] = -1000;
    tool.Rvec.ptr<float>(0)[0] = -1000;
    tool.Rvec.ptr<float>(0)[1] = -1000;
    tool.Rvec.ptr<float>(0)[2] = -1000;
    tool.ssize = -1000;
    return false;

}


float TrackTools::markerAera(int index)
{
    float A, B, C, S;
    float areaL, areaR, area_1, area_2;

    A = sqrt(pow(StereoMarkersL[index][0].x-StereoMarkersL[index][1].x,2) +
             pow(StereoMarkersL[index][0].y-StereoMarkersL[index][1].y,2));
    B = sqrt(pow(StereoMarkersL[index][1].x-StereoMarkersL[index][2].x,2) +
             pow(StereoMarkersL[index][1].y-StereoMarkersL[index][2].y,2));
    C = sqrt(pow(StereoMarkersL[index][2].x-StereoMarkersL[index][0].x,2) +
             pow(StereoMarkersL[index][2].y-StereoMarkersL[index][0].y,2));
    S = (A+B+C)/2;
    area_1 = sqrt(S * (S-A) * (S-B) * (S-C));

    A = sqrt(pow(StereoMarkersL[index][0].x-StereoMarkersL[index][3].x,2) +
             pow(StereoMarkersL[index][0].y-StereoMarkersL[index][3].y,2));
    B = sqrt(pow(StereoMarkersL[index][3].x-StereoMarkersL[index][2].x,2) +
             pow(StereoMarkersL[index][3].y-StereoMarkersL[index][2].y,2));
    S = (A+B+C)/2;
    area_2 = sqrt(S * (S-A) * (S-B) * (S-C));

    areaL = area_1 + area_2;

    A = sqrt(pow(StereoMarkersR[index][0].x-StereoMarkersR[index][1].x,2) +
             pow(StereoMarkersR[index][0].y-StereoMarkersR[index][1].y,2));
    B = sqrt(pow(StereoMarkersR[index][1].x-StereoMarkersR[index][2].x,2) +
             pow(StereoMarkersR[index][1].y-StereoMarkersR[index][2].y,2));
    C = sqrt(pow(StereoMarkersR[index][2].x-StereoMarkersR[index][0].x,2) +
             pow(StereoMarkersR[index][2].y-StereoMarkersR[index][0].y,2));
    S = (A+B+C)/2;
    area_1 = sqrt(S * (S-A) * (S-B) * (S-C));

    A = sqrt(pow(StereoMarkersR[index][0].x-StereoMarkersR[index][3].x,2) +
             pow(StereoMarkersR[index][0].y-StereoMarkersR[index][3].y,2));
    B = sqrt(pow(StereoMarkersR[index][3].x-StereoMarkersR[index][2].x,2) +
             pow(StereoMarkersR[index][3].y-StereoMarkersR[index][2].y,2));
    S = (A+B+C)/2;
    area_2 = sqrt(S * (S-A) * (S-B) * (S-C));

    areaL = area_1 + area_2;

    return areaL + areaR;
}


void TrackTools::drawStereoMarkers(cv::Mat3b frameL, cv::Mat3b frameR)
{
    // draw markers detected in the right frame
    for (unsigned int i=0;i<TheMarkersR.size();i++)
    {
        TheMarkersR[i].draw(frameR,Scalar(0,0,255),1);
    }

    for (unsigned int i=0;i<TheMarkersL.size();i++)
    {
        TheMarkersL[i].draw(frameL,Scalar(0,0,255),1);
//        CvDrawingUtils::draw3dAxis(frameL, TheMarkersL[i], TheCameraParameters);
    }

    for (unsigned int i=0;i<MarkersR_track.size();i++)
    {
        if (MarkersR_track[i].size() > 0 && MarkersR_track[i][0].x>0)
            MarkersR_track[i].draw(frameR,Scalar(0,0,255),1);
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
            Tools[i].ssize /= 1.5;
            CvDrawingUtils::draw3dAxis(frameL, Tools[i], StereoCameraParametersL);
        }
    }
}

void TrackTools::drawNeedle(cv::Mat3b frame, bool bool_frameL, cv::Mat ToolHNeedle, cv::Mat ToolHNeedleTip, int toolIndex)
{
    // For frame right -------------------
    Mat r_vec2 = Mat(3,1, CV_32F, float(0));
    Mat P2_ = Mat(3,3, CV_32F, float(0));
    Mat clHcr, crHn, crHp;
    clHcr = Mat::eye(4,4,CV_32F); crHn = Mat::eye(4,4,CV_32F); crHp = Mat::eye(4,4,CV_32F);
    cv::Mat T_homogeneous(4,1,cv::DataType<float>::type); // translation vector
    cv::decomposeProjectionMatrix(P2, P2_, r_vec2, T_homogeneous);
    clHcr.at<float>(0,3) = (float)T_homogeneous.at<double>(0,0)/T_homogeneous.at<double>(3,0);
    clHcr.at<float>(1,3) = (float)T_homogeneous.at<double>(1,0)/T_homogeneous.at<double>(3,0);
    clHcr.at<float>(2,3) = (float)T_homogeneous.at<double>(2,0)/T_homogeneous.at<double>(3,0);
    P2_.copyTo(StereoCameraParametersR.CameraMatrix);

    cv::Mat tHn, cHt, cHn;
    tHn = Mat::eye(4,4,CV_32F);cHt = Mat::eye(4,4,CV_32F);cHn = Mat::eye(4,4,CV_32F);
    Marker Needle, NeedleEnd;

    // Compute needle tip
    ToolHNeedle.copyTo(tHn);
    tHn.convertTo(tHn, CV_32F);

    cHt.at<float>(0,3) = Tools[toolIndex].Tvec.ptr<float>(0)[0];
    cHt.at<float>(1,3) = Tools[toolIndex].Tvec.ptr<float>(0)[1];
    cHt.at<float>(2,3) = Tools[toolIndex].Tvec.ptr<float>(0)[2];
    cv::Rodrigues(Tools[toolIndex].Rvec, cHt.colRange(0,3).rowRange(0,3));

    cHn = cHt * tHn;
    cv::Rodrigues(cHn.colRange(0,3).rowRange(0,3), Needle.Rvec);
    Needle.Tvec.ptr<float>(0)[0] = cHn.at<float>(0,3);
    Needle.Tvec.ptr<float>(0)[1] = cHn.at<float>(1,3);
    Needle.Tvec.ptr<float>(0)[2] = cHn.at<float>(2,3);
    Needle.ssize = 0.002;

    if (!bool_frameL)
    {
        crHn = clHcr.inv() * cHn;
        cv::Rodrigues(crHn.colRange(0,3).rowRange(0,3), Needle.Rvec);
        Needle.Tvec.ptr<float>(0)[0] = crHn.at<float>(0,3);
        Needle.Tvec.ptr<float>(0)[1] = crHn.at<float>(1,3);
        Needle.Tvec.ptr<float>(0)[2] = crHn.at<float>(2,3);
    }

    // Compute needle end ---------------------------------------
    ToolHNeedleTip.copyTo(tHn);
    tHn.convertTo(tHn, CV_32F);

    cHn = cHt * tHn;
    cv::Rodrigues(cHn.colRange(0,3).rowRange(0,3), NeedleEnd.Rvec);
    NeedleEnd.Tvec.ptr<float>(0)[0] = cHn.at<float>(0,3);
    NeedleEnd.Tvec.ptr<float>(0)[1] = cHn.at<float>(1,3);
    NeedleEnd.Tvec.ptr<float>(0)[2] = cHn.at<float>(2,3);
    NeedleEnd.ssize = 0.005;


    if (!bool_frameL)
    {
        crHn = clHcr.inv() * cHn;
        cv::Rodrigues(crHn.colRange(0,3).rowRange(0,3), NeedleEnd.Rvec);
        NeedleEnd.Tvec.ptr<float>(0)[0] = crHn.at<float>(0,3);
        NeedleEnd.Tvec.ptr<float>(0)[1] = crHn.at<float>(1,3);
        NeedleEnd.Tvec.ptr<float>(0)[2] = crHn.at<float>(2,3);
    }

    if (bool_frameL)
    {
//        CvDrawingUtils::draw3dAxis(frame, Needle, StereoCameraParametersL);
        CvDrawingUtils::draw3dAxis(frame, NeedleEnd, StereoCameraParametersL);
    }
    else
    {
//        CvDrawingUtils::draw3dAxis(frame, Needle, StereoCameraParametersR);
        CvDrawingUtils::draw3dAxis(frame, NeedleEnd, StereoCameraParametersR);
    }

}

//void TrackTools::drawNeedle(cv::Mat3b frame, bool bool_frameL, cv::Mat ToolLHNeedle, cv::Mat ToolLHNeedleTip)
//{
//    // For frame right -------------------
//    Mat r_vec2 = Mat(3,1, CV_32F, float(0));
//    Mat P2_ = Mat(3,3, CV_32F, float(0));
//    Mat clHcr, crHn, crHp;
//    clHcr = Mat::eye(4,4,CV_32F); crHn = Mat::eye(4,4,CV_32F); crHp = Mat::eye(4,4,CV_32F);
//    cv::Mat T_homogeneous(4,1,cv::DataType<float>::type); // translation vector
//    cv::decomposeProjectionMatrix(P2, P2_, r_vec2, T_homogeneous);
//    clHcr.at<float>(0,3) = (float)T_homogeneous.at<double>(0,0)/T_homogeneous.at<double>(3,0);
//    clHcr.at<float>(1,3) = (float)T_homogeneous.at<double>(1,0)/T_homogeneous.at<double>(3,0);
//    clHcr.at<float>(2,3) = (float)T_homogeneous.at<double>(2,0)/T_homogeneous.at<double>(3,0);
//    P2_.copyTo(StereoCameraParametersR.CameraMatrix);


//    //if(DrawNeedle)
//    {
//        cv::Mat lHn, cHl, cHn;
//        lHn = Mat::eye(4,4,CV_32F);cHl = Mat::eye(4,4,CV_32F);cHn = Mat::eye(4,4,CV_32F);
//        Marker Needle, NeedleEnd;

//        // Compute needle tip
//        ToolLHNeedleTip.copyTo(lHn);
//        lHn.convertTo(lHn, CV_32F);
////        lHn.at<float>(0,3) = 0.03194-0.001;
////        lHn.at<float>(1,3) = -0.00533;
////        lHn.at<float>(2,3) = 0.01319-0.0015;

//        cHl.at<float>(0,3) = Tools[1].Tvec.ptr<float>(0)[0];
//        cHl.at<float>(1,3) = Tools[1].Tvec.ptr<float>(0)[1];
//        cHl.at<float>(2,3) = Tools[1].Tvec.ptr<float>(0)[2];
//        cv::Rodrigues(Tools[1].Rvec, cHl.colRange(0,3).rowRange(0,3));

////        cout << "cHl " << cHl << endl;
////        cout << "lHn " << lHn << endl;

//        cHn = cHl * lHn;
//        cv::Rodrigues(cHn.colRange(0,3).rowRange(0,3), Needle.Rvec);
//        Needle.Tvec.ptr<float>(0)[0] = cHn.at<float>(0,3);
//        Needle.Tvec.ptr<float>(0)[1] = cHn.at<float>(1,3);
//        Needle.Tvec.ptr<float>(0)[2] = cHn.at<float>(2,3);
//        Needle.ssize = 0.005;

//        if (!bool_frameL)
//        {
//            crHn = clHcr.inv() * cHn;
//            cv::Rodrigues(crHn.colRange(0,3).rowRange(0,3), Needle.Rvec);
//            Needle.Tvec.ptr<float>(0)[0] = crHn.at<float>(0,3);
//            Needle.Tvec.ptr<float>(0)[1] = crHn.at<float>(1,3);
//            Needle.Tvec.ptr<float>(0)[2] = crHn.at<float>(2,3);
//        }

//        // Compute needle end ---------------------------------------
//        ToolLHNeedle.copyTo(lHn);
//        lHn.convertTo(lHn, CV_32F);

//        cHn = cHl * lHn;
//        cv::Rodrigues(cHn.colRange(0,3).rowRange(0,3), NeedleEnd.Rvec);
//        NeedleEnd.Tvec.ptr<float>(0)[0] = cHn.at<float>(0,3);
//        NeedleEnd.Tvec.ptr<float>(0)[1] = cHn.at<float>(1,3);
//        NeedleEnd.Tvec.ptr<float>(0)[2] = cHn.at<float>(2,3);
//        NeedleEnd.ssize = 0.005;


//        if (!bool_frameL)
//        {
//            crHn = clHcr.inv() * cHn;
//            cv::Rodrigues(crHn.colRange(0,3).rowRange(0,3), NeedleEnd.Rvec);
//            NeedleEnd.Tvec.ptr<float>(0)[0] = crHn.at<float>(0,3);
//            NeedleEnd.Tvec.ptr<float>(0)[1] = crHn.at<float>(1,3);
//            NeedleEnd.Tvec.ptr<float>(0)[2] = crHn.at<float>(2,3);
//        }

//        if (bool_frameL)
//        {
//            CvDrawingUtils::draw3dAxis(frame, Needle, StereoCameraParametersL);
//            CvDrawingUtils::draw3dAxis(frame, NeedleEnd, StereoCameraParametersL);
//        }
//        else
//        {
//            CvDrawingUtils::draw3dAxis(frame, Needle, StereoCameraParametersR);
//            CvDrawingUtils::draw3dAxis(frame, NeedleEnd, StereoCameraParametersR);
//        }
//    }
//}


void TrackTools::drawTip(cv::Mat3b frame, bool bool_frameL, cv::Mat ToolRHTipL, cv::Mat ToolRHTipR)
{
    // For frame right -------------------
    Mat r_vec2 = Mat(3,1, CV_32F, float(0));
    Mat P2_ = Mat(3,3, CV_32F, float(0));
    Mat clHcr, crHn, crHp;
    clHcr = Mat::eye(4,4,CV_32F); crHn = Mat::eye(4,4,CV_32F); crHp = Mat::eye(4,4,CV_32F);
    cv::Mat T_homogeneous(4,1,cv::DataType<float>::type); // translation vector
    cv::decomposeProjectionMatrix(P2, P2_, r_vec2, T_homogeneous);
    clHcr.at<float>(0,3) = (float)T_homogeneous.at<double>(0,0)/T_homogeneous.at<double>(3,0);
    clHcr.at<float>(1,3) = (float)T_homogeneous.at<double>(1,0)/T_homogeneous.at<double>(3,0);
    clHcr.at<float>(2,3) = (float)T_homogeneous.at<double>(2,0)/T_homogeneous.at<double>(3,0);
    P2_.copyTo(StereoCameraParametersR.CameraMatrix);


    //if(DrawNeedle)
    {
        cv::Mat rHp, cHr, cHp;
        rHp = Mat::eye(4,4,CV_32F);cHr = Mat::eye(4,4,CV_32F);cHp = Mat::eye(4,4,CV_32F);
        Marker TipL, TipR;

        // Compute tip left
        ToolRHTipL.copyTo(rHp);
        rHp.convertTo(rHp, CV_32F);

        cHr.at<float>(0,3) = Tools[2].Tvec.ptr<float>(0)[0];
        cHr.at<float>(1,3) = Tools[2].Tvec.ptr<float>(0)[1];
        cHr.at<float>(2,3) = Tools[2].Tvec.ptr<float>(0)[2];
        cv::Rodrigues(Tools[2].Rvec, cHr.colRange(0,3).rowRange(0,3));
        cHp = cHr * rHp;
        cv::Rodrigues(cHp.colRange(0,3).rowRange(0,3), TipL.Rvec);
        TipL.Tvec.ptr<float>(0)[0] = cHp.at<float>(0,3);
        TipL.Tvec.ptr<float>(0)[1] = cHp.at<float>(1,3);
        TipL.Tvec.ptr<float>(0)[2] = cHp.at<float>(2,3);
        TipL.ssize = 0.002;

        if (!bool_frameL)
        {
            crHp = clHcr.inv() * cHp;
            cv::Rodrigues(crHp.colRange(0,3).rowRange(0,3), TipL.Rvec);
            TipL.Tvec.ptr<float>(0)[0] = crHp.at<float>(0,3);
            TipL.Tvec.ptr<float>(0)[1] = crHp.at<float>(1,3);
            TipL.Tvec.ptr<float>(0)[2] = crHp.at<float>(2,3);
        }

        // Compute tip right
        ToolRHTipR.copyTo(rHp);
        rHp.convertTo(rHp, CV_32F);

        cHr.at<float>(0,3) = Tools[2].Tvec.ptr<float>(0)[0];
        cHr.at<float>(1,3) = Tools[2].Tvec.ptr<float>(0)[1];
        cHr.at<float>(2,3) = Tools[2].Tvec.ptr<float>(0)[2];
        cv::Rodrigues(Tools[2].Rvec, cHr.colRange(0,3).rowRange(0,3));
        cHp = cHr * rHp;
        cv::Rodrigues(cHp.colRange(0,3).rowRange(0,3), TipR.Rvec);
        TipR.Tvec.ptr<float>(0)[0] = cHp.at<float>(0,3);
        TipR.Tvec.ptr<float>(0)[1] = cHp.at<float>(1,3);
        TipR.Tvec.ptr<float>(0)[2] = cHp.at<float>(2,3);
        TipR.ssize = 0.002;

        if (!bool_frameL)
        {
            crHp = clHcr.inv() * cHp;
            cv::Rodrigues(crHp.colRange(0,3).rowRange(0,3), TipR.Rvec);
            TipR.Tvec.ptr<float>(0)[0] = crHp.at<float>(0,3);
            TipR.Tvec.ptr<float>(0)[1] = crHp.at<float>(1,3);
            TipR.Tvec.ptr<float>(0)[2] = crHp.at<float>(2,3);
        }

        if (bool_frameL)
        {
            CvDrawingUtils::draw3dAxis(frame, TipL, StereoCameraParametersL);
//            CvDrawingUtils::draw3dAxis(frame, TipR, StereoCameraParametersL);
        }
        else
        {
            CvDrawingUtils::draw3dAxis(frame, TipL, StereoCameraParametersR);
//            CvDrawingUtils::draw3dAxis(frame, TipR, StereoCameraParametersR);
        }
    }
}

void TrackTools::drawKalmans(cv::Mat3b frameL)
{
    for (int i=0; i<numTools; i++)
    {
        CvDrawingUtils::draw3dAxis(frameL, Tools_kalman[i], StereoCameraParametersL);
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
    trans.convertTo(trans, CV_32F);

    toolp.Tvec.ptr<float>(0)[0] = trans.at<float>(0,3);
    toolp.Tvec.ptr<float>(0)[1] = trans.at<float>(1,3);
    toolp.Tvec.ptr<float>(0)[2] = trans.at<float>(2,3);

    cv::Mat rvec;
    cv::Rodrigues(trans.colRange(0,3).rowRange(0,3), rvec);

    toolp.Tvec.convertTo(toolp.Tvec, CV_32F);
    rvec.convertTo(toolp.Rvec, CV_32F);
}


void TrackTools::initTrackers()
{
    // initialize marker array of tools --------------------------
    numMarkers_tool = 0;
    Marker tempMarker;
    tempMarker.Tvec.ptr<float>(0)[0] = -1000;
    tempMarker.Tvec.ptr<float>(0)[1] = -1000;
    tempMarker.Tvec.ptr<float>(0)[2] = -1000;
    tempMarker.Rvec.ptr<float>(0)[0] = -1000;
    tempMarker.Rvec.ptr<float>(0)[1] = -1000;
    tempMarker.Rvec.ptr<float>(0)[2] = -1000;

    for (int i=1; i<ToolsID.size(); i++)
    {// exclude Mandrel: tool 0
        for (int j=0; j<ToolsID[i].size(); j++)
        {
            tempMarker.id = ToolsID[i][j];
            MarkersL_track.push_back(tempMarker);
            MarkersR_track.push_back(tempMarker);
            numMarkers_tool ++;
        }
    }

    // initialization of trackers ------------------------------------
    vector<cv::Point2f> markercornerLocal;
    markercornerLocal.resize(4);

    TheMarkerSize = MarkersSize[1]; // exclude Mandrel: tool 0

    double halfSize=TheMarkerSize/2.;
    markercornerLocal[0].x = -halfSize;
    markercornerLocal[0].y = -halfSize;
    markercornerLocal[1].x = -halfSize;
    markercornerLocal[1].y = halfSize;
    markercornerLocal[2].x = halfSize;
    markercornerLocal[2].y = halfSize;
    markercornerLocal[3].x = halfSize;
    markercornerLocal[3].y = -halfSize;

    //vector<dvrk::TrackerGeneral> trackersL, trackersR;
    for (int i=0; i<numMarkers_tool; i++)
    {
        dvrk::TrackerGeneral traker_tmp(markercornerLocal);
        trackersL.push_back(traker_tmp); //Tracker for left image
        trackersR.push_back(traker_tmp); //Tracker for right image
    }

    // initialization of filter ------------------------
    if (filterwindow > 0)
    {
        MarkersL_hist.resize(numMarkers_tool);
        MarkersR_hist.resize(numMarkers_tool);
    }
}


vector<bool> TrackTools::trackMarkers(cv::Mat &track_image, vector<Marker> &detectedMarkers,
                          vector<Marker> &toolMarkers, vector<dvrk::TrackerGeneral> &trackers,
                          CameraParameters camParams ,float markerSizeMeters)
{
    vector<bool> markerDetected;
    for (int i=0; i<numMarkers_tool; i++)
    {
        markerDetected.push_back(false);
    }
    // Loop detected markers, fill in tool markers
    for (int i=0; i<toolMarkers.size(); i++)
    {
        for (int j=0; j<detectedMarkers.size(); j++)
        {
            if (toolMarkers[i].id == detectedMarkers[j].id)
            {
                toolMarkers[i].clear();
                toolMarkers[i].Tvec.ptr<float>(0)[0] = detectedMarkers[j].Tvec.ptr<float>(0)[0];
                toolMarkers[i].Tvec.ptr<float>(0)[1] = detectedMarkers[j].Tvec.ptr<float>(0)[1];
                toolMarkers[i].Tvec.ptr<float>(0)[2] = detectedMarkers[j].Tvec.ptr<float>(0)[2];
                toolMarkers[i].Rvec.ptr<float>(0)[0] = detectedMarkers[j].Rvec.ptr<float>(0)[0];
                toolMarkers[i].Rvec.ptr<float>(0)[1] = detectedMarkers[j].Rvec.ptr<float>(0)[1];
                toolMarkers[i].Rvec.ptr<float>(0)[2] = detectedMarkers[j].Rvec.ptr<float>(0)[2];

                for (int k=0; k<4; k++)
                {//Copy corners
                    toolMarkers[i].push_back(detectedMarkers[j][k]);
                }
                markerDetected[i]=true;
                //cout << toolMarkers[i].id << " " ;
                break;
            }
        }

    }
   // cout << endl;
    // track if not detected
    for (int i=0; i<trackers.size(); i++)
    {
        if (toolMarkers[i].Tvec.ptr<float>(0)[0] != -1000)
        {// if marker has been found before
            vector<cv::Point2f> corners_before, corners_after;
            cv::Mat tvec_before;
            toolMarkers[i].Tvec.copyTo(tvec_before);
            for (int k=0; k<4; k++)
            {
                corners_before.push_back(toolMarkers[i][k]);
                corners_after.push_back(toolMarkers[i][k]);
            }
            bool before, after;
            before = markerDetected[i];

            // Tracking -------
            markerDetected[i] = trackers[i].track(track_image, corners_after, markerDetected[i]);
            // ----------------

            after = markerDetected[i];

            if(markerDetected[i])
            {//If marker is detected or tracked
                for ( int c=0;c<4;c++ )
                    toolMarkers[i][c]=corners_after[c];

                toolMarkers[i].calculateExtrinsics(markerSizeMeters,camParams); //Update Tvec, Rvec
            }

            if (before == false && after == true)
            {// if marker is tracked;
                float norm0 = cv::norm(cv::Mat(corners_before[0]),cv::Mat(corners_after[0]));
                float norm1 = cv::norm(cv::Mat(corners_before[1]),cv::Mat(corners_after[1]));
                float norm2 = cv::norm(cv::Mat(corners_before[2]),cv::Mat(corners_after[2]));
                float norm3 = cv::norm(cv::Mat(corners_before[3]),cv::Mat(corners_after[3]));
                //cout << norm0 << " " << norm1 << " " << norm2 << " " << norm3 << endl;
                if (norm0<7 && norm1<7 && norm2<7 && norm3<7)
                {}
                else
                {///////// too fast, failed tracking ///////////
                    markerDetected[i] = false;
                }

            }
        }
    }

    return markerDetected;
}



/************************************
 *
 *
 *
 *kalmanFilterTrans
 ************************************/

void TrackTools::initializeKF(cv::KalmanFilter &KF, cv::KalmanFilter &KF_rot)
{

//    KF.transitionMatrix = *(Mat_<float>(6,6)
//    <<1,0,0,10,0,0,  0,1,0,0,10,0, 0,0,1,0,0,10, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1);

    KF.transitionMatrix = (Mat_<float>(6,6)
            <<1,0,0,10,0,0,  0,1,0,0,10,0, 0,0,1,0,0,10, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1);
    KF.statePre.at<float>(3) = 0;
    KF.statePre.at<float>(4) = 0;
    KF.statePre.at<float>(5) = 0;
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-1));//e-4));
    setIdentity(KF.measurementNoiseCov, Scalar::all(10));//10
    setIdentity(KF.errorCovPost, Scalar::all(.1));

//    KF_rot.transitionMatrix = *(Mat_<float>(6,6)
//            <<1,0,0,10,0,0,  0,1,0,0,10,0, 0,0,1,0,0,10, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1);

    KF_rot.transitionMatrix = (Mat_<float>(6,6)
            <<1,0,0,10,0,0,  0,1,0,0,10,0, 0,0,1,0,0,10, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1);
    KF_rot.statePre.at<float>(3) = 0;
    KF_rot.statePre.at<float>(4) = 0;
    KF_rot.statePre.at<float>(5) = 0;
    setIdentity(KF_rot.measurementMatrix);
    setIdentity(KF_rot.processNoiseCov, Scalar::all(1e-1));//e-4));MandrelDetected
    setIdentity(KF_rot.measurementNoiseCov, Scalar::all(10));//10
    setIdentity(KF_rot.errorCovPost, Scalar::all(.1));
}


void TrackTools::kalmanFilterPose(Marker &pose_detected, Marker &pose_previous,
                                  Marker &pose_filtered,
                                  cv::KalmanFilter &KF, cv::KalmanFilter &KF_rot,
                                  int tool_ind)
{
    // Initialise KF -----------
    if (fristDetected[tool_ind])
    {
        fristDetected[tool_ind] = false;

        KF.statePre.at<float>(0) = pose_detected.Tvec.ptr<float>(0)[0];
        KF.statePre.at<float>(1) = pose_detected.Tvec.ptr<float>(0)[1];
        KF.statePre.at<float>(2) = pose_detected.Tvec.ptr<float>(0)[2];

        KF_rot.statePre.at<float>(0) = pose_detected.Rvec.ptr<float>(0)[0];
        KF_rot.statePre.at<float>(1) = pose_detected.Rvec.ptr<float>(0)[1];
        KF_rot.statePre.at<float>(2) = pose_detected.Rvec.ptr<float>(0)[2];
    }

    // Prediction by KF
    Mat prediction = KF.predict();
    Mat prediction_rot = KF_rot.predict();

    // If tool detected
    if (pose_detected.Rvec.ptr<float>(0)[0] != -1000)
    {
        float theta = norm(pose_detected.Rvec);

        if ((pose_previous.Rvec.dot(pose_detected.Rvec)) < 0)
        {
            pose_detected.Rvec.ptr<float>(0)[0] = (1-3.14*2/theta) * pose_detected.Rvec.ptr<float>(0)[0];
            pose_detected.Rvec.ptr<float>(0)[1] = (1-3.14*2/theta) * pose_detected.Rvec.ptr<float>(0)[1];
            pose_detected.Rvec.ptr<float>(0)[2] = (1-3.14*2/theta) * pose_detected.Rvec.ptr<float>(0)[2];
            //cout << " reverse! ----" << endl;
        }

        Mat estimated = KF.correct(pose_detected.Tvec);
        pose_filtered.Tvec.ptr<float>(0)[0] = estimated.at<float>(0);
        pose_filtered.Tvec.ptr<float>(0)[1] = estimated.at<float>(1);
        pose_filtered.Tvec.ptr<float>(0)[2] = estimated.at<float>(2);

        Mat estimated_rot = KF_rot.correct(pose_detected.Rvec);
        pose_filtered.Rvec.ptr<float>(0)[0] = estimated_rot.at<float>(0);
        pose_filtered.Rvec.ptr<float>(0)[1] = estimated_rot.at<float>(1);
        pose_filtered.Rvec.ptr<float>(0)[2] = estimated_rot.at<float>(2);
        //pose_detected.Rvec.copyTo(pose_filtered.Rvec);

        pose_filtered.ssize = pose_detected.ssize;
    }

    // If tool lost -------
    else if(!fristDetected[tool_ind])
    {
        pose_previous.Tvec.copyTo(pose_detected.Tvec);
        pose_previous.Rvec.copyTo(pose_detected.Rvec);

//        cout << "pose_previous.Tvec "
//             << " " <<  pose_previous.Tvec.ptr<float>(0)[0]
//             << " " <<  pose_previous.Tvec.ptr<float>(0)[1]
//             << " " <<  pose_previous.Tvec.ptr<float>(0)[2]
//             << endl;

//        cout << "pose_filtered.Tvec "
//             << " " <<  pose_filtered.Tvec.ptr<float>(0)[0]
//             << " " <<  pose_filtered.Tvec.ptr<float>(0)[1]
//             << " " <<  pose_filtered.Tvec.ptr<float>(0)[2]
//             << endl;

        float theta = norm(pose_detected.Rvec);

        pose_filtered.Tvec.ptr<float>(0)[0] = prediction.at<float>(0);
        pose_filtered.Tvec.ptr<float>(0)[1] = prediction.at<float>(1);
        pose_filtered.Tvec.ptr<float>(0)[2] = prediction.at<float>(2);

        pose_filtered.Rvec.ptr<float>(0)[0] = prediction_rot.at<float>(0);
        pose_filtered.Rvec.ptr<float>(0)[1] = prediction_rot.at<float>(1);
        pose_filtered.Rvec.ptr<float>(0)[2] = prediction_rot.at<float>(2);

        if ((pose_previous.Rvec.dot(pose_filtered.Rvec)) < 0)
        {
            pose_filtered.Rvec.ptr<float>(0)[0] = (1-3.14*2/theta) * pose_filtered.Rvec.ptr<float>(0)[0];
            pose_filtered.Rvec.ptr<float>(0)[1] = (1-3.14*2/theta) * pose_filtered.Rvec.ptr<float>(0)[1];
            pose_filtered.Rvec.ptr<float>(0)[2] = (1-3.14*2/theta) * pose_filtered.Rvec.ptr<float>(0)[2];
            //cout << " no detection! reverse! ----" << endl;
        }

        pose_filtered.ssize = pose_previous.ssize;
    }
}
