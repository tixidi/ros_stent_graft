#include"visualTrackingThread.h"

using namespace myaruco;

VisualTrackingThread::VisualTrackingThread()
{
#ifndef SimulationON
    camera.Left.open(0);
    camera.Right.open(1);
#endif
    if( !camera.Left.isOpened() )
    {
        cout << "Reading left.png!" << endl;
    }
    else
    {
        camera.Left.set(CV_CAP_PROP_FPS,20);
    }

    if( ! camera.Right.isOpened() )
    {
        cout << "Reading right.png!" << endl;
    }
    else
    {
        camera.Right.set(CV_CAP_PROP_FPS,20);
    }

   //string CAMERA_CALI_DIR = "/home/yang/CameraCalibration/";


   trackingNeedleDriver=new NeedleDriverTracking();
   needleTracking= new NeedlePoseEstimation();

   disPlayTime=new QTime();
   disPlayTime->start();   

   // Read hand eye cali grid points ------------------------------------
   ifstream f_rframe(SRC_FILES_DIR+"VisionSystem/caliInfo/handeyeCaliInput/robotframe.txt");
   ifstream f_cframe(SRC_FILES_DIR+"VisionSystem/caliInfo/handeyeCaliInput/cameraframe.txt");
   string xyz;


       for (int i=0; i<GRID_1; i++)
       {
               vector<vector<cv::Point3d> > tmpvvR, tmpvvC;
               grid_robot.push_back(tmpvvR);
               grid_camera.push_back(tmpvvC);

               for (int j=0; j<GRID_2; j++)
               {
                   vector<cv::Point3d> tmpvR, tmpvC;
                   grid_robot[i].push_back(tmpvR);
                   grid_camera[i].push_back(tmpvC);

                   for (int k=0; k<GRID_3; k++)
                   {
                       cv::Point3d tmpR, tmpC;
                       grid_robot[i][j].push_back(tmpR);
                       grid_camera[i][j].push_back(tmpC);

                       f_rframe >> xyz;
                       grid_robot[i][j][k].x = atof(xyz.c_str());
                       f_rframe >> xyz;
                       grid_robot[i][j][k].y = atof(xyz.c_str());
                       f_rframe >> xyz;
                       grid_robot[i][j][k].z = atof(xyz.c_str());
                       f_cframe >> xyz;
                       grid_camera[i][j][k].x = atof(xyz.c_str());
                       f_cframe >> xyz;
                       grid_camera[i][j][k].y = atof(xyz.c_str());
                       f_cframe >> xyz;
                       grid_camera[i][j][k].z = atof(xyz.c_str());
                   }
               }
      }

    sigREcod=0;
//   cv:Mat image= camera
//   VideoWriter video("out.avi",CV_FOURCC('M','J','P','G'),10, Size(frame.size().width, frame.size().height),true);


    // Read camera parameters
    FileStorage f_M1(CAMERA_CALIB_DIR + "M1.xml", FileStorage::READ); f_M1["M1"]>>M1;
    FileStorage f_D1(CAMERA_CALIB_DIR + "D1.xml", FileStorage::READ); f_D1["D1"]>>D1;
    FileStorage f_mx1(CAMERA_CALIB_DIR + "mx1.xml", FileStorage::READ); f_mx1["mx1"]>>mx1;
    FileStorage f_my1(CAMERA_CALIB_DIR + "my1.xml", FileStorage::READ); f_my1["my1"]>>my1;
    FileStorage f_P1(CAMERA_CALIB_DIR + "P1.xml", FileStorage::READ); f_P1["P1"]>>P1;

    FileStorage f_M2(CAMERA_CALIB_DIR + "M2.xml", FileStorage::READ); f_M2["M2"]>>M2;
    FileStorage f_D2(CAMERA_CALIB_DIR + "D2.xml", FileStorage::READ); f_D2["D2"]>>D2;
    FileStorage f_mx2(CAMERA_CALIB_DIR + "mx2.xml", FileStorage::READ); f_mx2["mx2"]>>mx2;
    FileStorage f_my2(CAMERA_CALIB_DIR + "my2.xml", FileStorage::READ); f_my2["my2"]>>my2;
    FileStorage f_P2(CAMERA_CALIB_DIR + "P2.xml", FileStorage::READ); f_P2["P2"]>>P2;

    // TrackTools --------
    DrawTool = false;
    //string fileDir=SRC_FILES_DIR+"VisionSystem/tracktools/";

    toolsTracker = new TrackTools(CAMERA_CALIB_DIR);
    filterWindow = 0;
    fCOUNTER = 1;

    // Read needle points -------------
    ifstream f_streamNshape1(SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/NeedleShape_9x17_1.txt");
    char  line[1024]={0};
    string nx, ny, nz;

    while(f_streamNshape1.getline(line, sizeof(line)))
    {
        std::stringstream  word(line);
        word >> nx; word >> ny; word >> nz;
        cv::Point3d nPnt;
        nPnt.x = atof(nx.c_str());
        nPnt.y = atof(ny.c_str());
        nPnt.z = atof(nz.c_str());

        NeedlePoints1.push_back(nPnt);
    }

    ifstream f_streamNshape2(SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/NeedleShape_9x17_2.txt");
    //char  line[1024]={0};
    //string nx, ny, nz;

    while(f_streamNshape2.getline(line, sizeof(line)))
    {
        std::stringstream  word(line);
        word >> nx; word >> ny; word >> nz;
        cv::Point3d nPnt;
        nPnt.x = atof(nx.c_str());
        nPnt.y = atof(ny.c_str());
        nPnt.z = atof(nz.c_str());

        NeedlePoints2.push_back(nPnt);
    }

//    for (int i=0; i<11; i++)
//     {
//         double variable;
//         cv::Point3d nPnt;
//         f_streamNshape >> variable;
//         nPnt.x = variable;
//         f_streamNshape >> variable;
//         nPnt.y = variable;
//         f_streamNshape >> variable;
//         nPnt.z = variable;

//         NeedlePoints.push_back(nPnt);
//     }

     // Read needle to tool pose -----------------------------
    ToolLHNeedle = Mat::eye(4,4,CV_64F);
    ifstream f_streamlHn(SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/ToolLHNeedleEnd_9x17.txt");
      for (int i=0; i<4; i++)
          for (int j=0; j<4; j++)
              {
                  double variable;
                  f_streamlHn >> variable;
                  ToolLHNeedle.at<double>(i,j)=variable;
              }
      //cout << "ToolLHNeedle " << ToolLHNeedle << endl;

      ToolLHNeedleC = Mat::eye(4,4,CV_64F);
      ifstream f_streamlHnc(SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/ToolLHNeedleCentre.txt");
        for (int i=0; i<4; i++)
            for (int j=0; j<4; j++)
                {
                    double variable;
                    f_streamlHnc >> variable;
                    ToolLHNeedleC.at<double>(i,j)=variable;
                }
        //cout << "ToolLHNeedleCentre " << ToolLHNeedleC << endl;

      ToolLHNeedleTip = Mat::eye(4,4,CV_64F);
      ifstream f_streamlHt(SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/ToolLHNeedleTip.txt");
        for (int i=0; i<4; i++)
            for (int j=0; j<4; j++)
                {
                    double variable;
                    f_streamlHt >> variable;
                    ToolLHNeedleTip.at<double>(i,j)=variable;
                }
        //cout << "ToolLHNeedleTip " << ToolLHNeedleTip << endl;


        tHn_ = Mat::eye(4,4,CV_64F);
//        ifstream f_tHn_(SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/tHn_.txt");
//          for (int i=0; i<4; i++)
//              for (int j=0; j<4; j++)
//                  {
//                      double variable;
//                      f_tHn_ >> variable;
//                      tHn_.at<double>(i,j)=variable;
//                  }
          //cout << "tHn_ " << tHn_ << endl;

        ToolRHTipL = Mat::eye(4,4,CV_64F);
        ifstream f_streamRHl(SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/ToolRHTipLeft.txt");
          for (int i=0; i<4; i++)
              for (int j=0; j<4; j++)
                  {
                      double variable;
                      f_streamRHl >> variable;
                      ToolRHTipL.at<double>(i,j)=variable;
                  }
          //cout << "ToolRHTipL " << ToolRHTipL << endl;

          ToolRHTipR = Mat::eye(4,4,CV_64F);
          ifstream f_streamRHr(SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/ToolRHTipRight.txt");
            for (int i=0; i<4; i++)
                for (int j=0; j<4; j++)
                    {
                        double variable;
                        f_streamRHr >> variable;
                        ToolRHTipR.at<double>(i,j)=variable;
                    }
            //cout << "ToolRHTipR " << ToolRHTipR << endl;

        ToolRHNeedle = Mat::eye(4,4,CV_64F);
        ifstream f_streamRHn(SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/ToolRHNeedleEnd_9x17.txt");
          for (int i=0; i<4; i++)
              for (int j=0; j<4; j++)
                  {
                      double variable;
                      f_streamRHn >> variable;
                      ToolRHNeedle.at<double>(i,j)=variable;
                  }
          //cout << "ToolRHNeedle " << ToolRHNeedle << endl;

          ToolRHNeedleTip = Mat::eye(4,4,CV_64F);
          ifstream f_streamRHt(SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/ToolRHNeedleTip.txt");
            for (int i=0; i<4; i++)
                for (int j=0; j<4; j++)
                    {
                        double variable;
                        f_streamRHt >> variable;
                        ToolRHNeedleTip.at<double>(i,j)=variable;
                    }
            //cout << "ToolRHNeedleTip " << ToolRHNeedleTip << endl;

      needleDetected = false;
      saveDetecedNeedle = false;


      // Counter for frames ----------
      countFrames = 0;

      // Save image ------------------
      imageInd = 1;
      Traj_index = -1;

      // Timer -----------------------
      bool_recordTime = false;
      imageCnt  = 0;
      //toolsInCamTimer.open(SRC_FILES_DIR+"demonstrantion/toolsTimer.txt");

      // Force -----------------------
      DrawForce = false;
      forceX = 0; forceY = 0; forceZ = 0;

      // Inspection ---------------
      Inspect = true;
      ename_needleEnergyL = "eng_l.png";
      ename_needleEnergyR = "eng_r.png";

      // Detect thread ---------------
      fname_thread_ori = strdup((SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/thread_pose_ori.txt").c_str());
      fname_thread_new = strdup((SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/thread_pose_new.txt").c_str());
}

void VisualTrackingThread::run()
{
    cv::Mat frameL, frameR;
    cv::Mat frameLeft_rect, frameRight_rect;
    cv::Mat3b hardimageL, hardimageR;

    hardimageL = imread(SRC_FILES_DIR+"VisionSystem/left1.png", CV_LOAD_IMAGE_COLOR);
    hardimageR = imread(SRC_FILES_DIR+"VisionSystem/right1.png", CV_LOAD_IMAGE_COLOR);
    while (true)
    {
#ifdef SimulationON
        hardimageL.copyTo(frameL);
        hardimageR.copyTo(frameR);

        if( !camera.Left.isOpened() )
        {
            hardimageL.copyTo(frameL);
            hardimageR.copyTo(frameR);
        }
        else
        {
            camera.Left >> GlobaleImages.frameLeft;
            camera.Right>> GlobaleImages.frameRight;
            GlobaleImages.frameLeft.copyTo(frameL);
            GlobaleImages.frameRight.copyTo(frameR);
            countFrames ++;
        }

#else
//        hardimageL.copyTo(frameL);
//        hardimageR.copyTo(frameR);

        if( !camera.Left.isOpened() || !camera.Right.isOpened())
        {
            hardimageL.copyTo(frameL);
            hardimageR.copyTo(frameR);
            cout << "Cameras are not open!"<<endl;
        }
        else
        {
            camera.Left >> GlobaleImages.frameLeft;
            camera.Right>> GlobaleImages.frameRight;
            GlobaleImages.frameLeft.copyTo(frameL);
            GlobaleImages.frameRight.copyTo(frameR);
            //countFrames ++;
        }
#endif
        // Rectify images -------------------------------------------------------------
        frameLeft_rect.create( frameL.size(), frameLeft_rect.type() );
        frameRight_rect.create( frameR.size(), frameRight_rect.type() );
        remap(frameL, frameLeft_rect, mx1, my1, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );
        remap(frameR, frameRight_rect, mx2, my2, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );
        frameLeft_rect.copyTo(GlobaleImages.frameLeft);
        frameRight_rect.copyTo(GlobaleImages.frameRight);


//        frameL.copyTo(GlobaleImages.frameLeft);
//        frameR.copyTo(GlobaleImages.frameRight);


        DrawTool = true;
//       cout << "drawTool ==" << DrawTool<< endl;
//        DrawNeedle = false;
        // Draw tools -------------------
       if (DrawTool)
        {

            //msleep(150);
                detectToolsInCamFrame_global(frameLeft_rect, frameRight_rect);
                //detectToolsInCamFrame_global(frameL, frameR);
                if (bool_recordTime)
                {
                    for(int k=0; k<3; k++)
                    {
                        toolsInCamTimer << toolsTracker->Tools[k].Tvec.ptr<float>(0)[0] << " "
                                   << toolsTracker->Tools[k].Tvec.ptr<float>(0)[1] << " "
                                   << toolsTracker->Tools[k].Tvec.ptr<float>(0)[2] << " "
                                   << toolsTracker->Tools[k].Rvec.ptr<float>(0)[0] << " "
                                   << toolsTracker->Tools[k].Rvec.ptr<float>(0)[1] << " "
                                   << toolsTracker->Tools[k].Rvec.ptr<float>(0)[2] << " ";
                    }

                    timer = time(0);   // get time now
                    struct tm * now = localtime( & timer );
                    char buffer [80];
                    strftime (buffer,80,"%Y %m %d %H %M %S ",now);
                    struct timeval tp;
                    gettimeofday(&tp, NULL);
                    long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds

                    toolsInCamTimer << buffer << ms << endl;
                }
        }

         DrawTrajectory = false;

        // Draw trajectory -------------------------

         if (DrawMandrel && toolLTraj_tvec.size()>0){
             //draw seing slot
             drawTrajectory(mandrel_tvec,255,255,0);
         }


        if (DrawTrajectoryToolL && toolLTraj_tvec.size()>0)
        {

						ofstream drawToolLTraj("/home/charlie/Documents/workspace/matlab/drawToolLTraj.txt");
						for(int i = 0; i < toolLTraj_tvec.size();i++){
								drawToolLTraj<<toolLTraj_tvec[i].x<<" "<<toolLTraj_tvec[i].y<<" "<<toolLTraj_tvec[i].z<<" "<<endl;
						}
            drawTrajectory(toolLTraj_tvec, 255, 255, 0);

//            drawNeedleTrajectory(toolLTraj_tvec, toolLTraj_rvec, 255, 0, 255);
        }
        if (DrawTrajectoryToolR && toolRTraj_tvec.size()>0)
        {
            drawTrajectory(toolRTraj_tvec, 0, 255, 255);
            //drawTipTrajectory(toolRTraj_tvec, toolRTraj_rvec, 255, 0, 0);
        }

        // Draw hand eye --------------------
//        DrawHandEye = true;
        if (DrawHandEye)
        {
//            showHandEye(frameLeft_rect, markerInHandEye);
//            showHandEye(frameLeft_rect, markerInHandEye_updated);

//            Functions::ToolPose2CVMat(toolsTracker->Tools[1]).copyTo(markerInHandEye);
            showHandEye(GlobaleImages.frameLeft, markerInHandEye_updated);
//            cout << "markerInHandEye_updated " << markerInHandEye_updated << endl;
        }

        drawSlots = true;
        if (drawSlots)
        {
            showSlots();
//            cout << "showing " << MAN_H_SLOTS_NEW_draw.size() << " slots!" << endl;
        }

/*        DrawForce = false;
        if (DrawForce)
        {
            Point3f a(0.0532107,  0.0036869,  0.155462);
            vector<Point3f> as;
            as.push_back(a);
            drawTrajectory(as, 255, 0, 255);
//            drawThreadForce(0.0658156, -0.0216693, 0.173914, forceX, forceY, forceZ);
        }

        if (PutText)
        {
            char text[255];
            sprintf(text, "%d", Traj_index);
            int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontScale = 1;
            int thickness = 3;
            cv::Point textOrg(500, 440);
            cv::putText(GlobaleImages.frameLeft, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness,8);
            cv::putText(GlobaleImages.frameRight, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness,8);
        }
*/
//        detectNeedle = true;
//        toolIndx_detectneedle = 2; // 1 or 2
/*        if (detectNeedle )
        {            
            NeedlePoints.clear();
            if (toolIndx_detectneedle == 1)
            {
                for (int i=0; i<NeedlePoints1.size(); i++)
                {
                    NeedlePoints.push_back(NeedlePoints1[i]);
                }
                fneedleNewInToolL.open(SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/lHn_.txt");
            }
            else
            {
                for (int i=0; i<NeedlePoints2.size(); i++)
                {
                    NeedlePoints.push_back(NeedlePoints2[i]);
                }
                fneedleNewInToolR.open(SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/rHn_.txt");
            }

            needlePoseEstimation * pose3Dto2D = new needlePoseEstimation();
            cv::Mat needlePoseFinal = cv::Mat::eye(4,4,CV_64F);
            Mat leftImage_gray;
            Mat rightImage_gray;
            vector<cv::Point3d> NeedlePoints_opt,NeedlePoints_cal;

            cvtColor(frameRight_rect, rightImage_gray, CV_RGB2GRAY);
            cvtColor(frameLeft_rect, leftImage_gray, CV_RGB2GRAY);
            IplImage currFrame1cLeft;
            IplImage currFrame1cRight;

            currFrame1cLeft = leftImage_gray;
            currFrame1cRight = rightImage_gray;

            D1.copyTo(pose3Dto2D->D1);
            D2.copyTo(pose3Dto2D->D2);
            P1.copyTo(pose3Dto2D->P1);
            P2.copyTo(pose3Dto2D->P2);

            //Bidan
            ToolLHNeedle.copyTo(pose3Dto2D->lHn);
            ToolRHNeedle.copyTo(pose3Dto2D->rHn);

            //cHl
            pose3Dto2D->cHl = cv::Mat::eye(4,4,CV_64F);
            Mat temp_now = Mat(3, 1, CV_64F, double(0));

            pose3Dto2D->cHl.at<double>(0,3) = (double)toolsTracker->Tools[1].Tvec.ptr<float>(0)[0];
            pose3Dto2D->cHl.at<double>(1,3) = (double)toolsTracker->Tools[1].Tvec.ptr<float>(0)[1];
            pose3Dto2D->cHl.at<double>(2,3) = (double)toolsTracker->Tools[1].Tvec.ptr<float>(0)[2];

            temp_now.at<double>(0,0) = (double)toolsTracker->Tools[1].Rvec.ptr<float>(0)[0];
            temp_now.at<double>(1,0) = (double)toolsTracker->Tools[1].Rvec.ptr<float>(0)[1];
            temp_now.at<double>(2,0) = (double)toolsTracker->Tools[1].Rvec.ptr<float>(0)[2];
            cv::Rodrigues(temp_now, pose3Dto2D->cHl.colRange(0,3).rowRange(0,3));

            //cHr
            pose3Dto2D->cHr = cv::Mat::eye(4,4,CV_64F);

            pose3Dto2D->cHr.at<double>(0,3) = (double)toolsTracker->Tools[2].Tvec.ptr<float>(0)[0];
            pose3Dto2D->cHr.at<double>(1,3) = (double)toolsTracker->Tools[2].Tvec.ptr<float>(0)[1];
            pose3Dto2D->cHr.at<double>(2,3) = (double)toolsTracker->Tools[2].Tvec.ptr<float>(0)[2];

            temp_now.at<double>(0,0) = (double)toolsTracker->Tools[2].Rvec.ptr<float>(0)[0];
            temp_now.at<double>(1,0) = (double)toolsTracker->Tools[2].Rvec.ptr<float>(0)[1];
            temp_now.at<double>(2,0) = (double)toolsTracker->Tools[2].Rvec.ptr<float>(0)[2];
            cv::Rodrigues(temp_now, pose3Dto2D->cHr.colRange(0,3).rowRange(0,3));

            cout<<"pose3Dto2D->cHl " << pose3Dto2D->cHl<<endl;


            vector<cv::Point3d> NeedlePoints_tmp;

            Mat nHp = Mat::eye(4,4,CV_64F);
            Mat tHp;
            for (int i=0; i<NeedlePoints.size(); i++)
            {
                NeedlePoints_tmp.push_back(NeedlePoints[i]);

                nHp.at<double>(0,3) = NeedlePoints[i].x;
                nHp.at<double>(1,3) = NeedlePoints[i].y;
                nHp.at<double>(2,3) = NeedlePoints[i].z;

                if (toolIndx_detectneedle == 1)
                    tHp = ToolLHNeedle * nHp;
                else
                    tHp = ToolRHNeedle * nHp;

                NeedlePoints_opt.push_back(NeedlePoints[i]);
                NeedlePoints_opt[i].x = tHp.at<double>(0,3);
                NeedlePoints_opt[i].y = tHp.at<double>(1,3);
                NeedlePoints_opt[i].z = tHp.at<double>(2,3);
            }

            cout << "ToolRHNeedle " << endl << ToolRHNeedle << endl;
			
			cout<<toolsTracker->Tools.size()<<endl;
			cout<<toolIndx_detectneedle<<endl;
            if (toolsTracker->Tools[toolIndx_detectneedle].Tvec.ptr<float>(0)[0] != -1000)
            {
				
                pose3Dto2D->estimatePose(&currFrame1cLeft, &currFrame1cRight, NeedlePoints_tmp, &needlePoseFinal, toolIndx_detectneedle);
//                pose3Dto2D->printNeedleSearchRange(GlobaleImages.frameLeft, GlobaleImages.frameRight, NeedlePoints,
//                                                   toolIndx_detectneedle, 0, 255, 0);

                needlePoseFinal.copyTo(tHn_);
                cout << "Detected needle pose tHn_ " << endl << tHn_ << endl;
                for (int i=0; i<4; i++)
                {
                    for (int j=0; j<4; j++)
                        {
                            if (toolIndx_detectneedle == 1)
                                fneedleNewInToolL << tHn_.at<double>(i,j) << " ";
                            else
                                fneedleNewInToolR << tHn_.at<double>(i,j) << " ";
                        }

                    if (toolIndx_detectneedle == 1)
                        fneedleNewInToolL << endl;
                    else
                        fneedleNewInToolR << endl;
					
                }
                if (toolIndx_detectneedle == 1)
                    fneedleNewInToolL.close();
                else
                    fneedleNewInToolR.close();
				
                for (int i=0; i<NeedlePoints.size(); i++)
                {
                    nHp.at<double>(0,3) = NeedlePoints[i].x;
                    nHp.at<double>(1,3) = NeedlePoints[i].y;
                    nHp.at<double>(2,3) = NeedlePoints[i].z;
                    tHp = needlePoseFinal * nHp;
                    NeedlePoints_cal.push_back(NeedlePoints_tmp[i]);
//                    NeedlePoints_cal.push_back(NeedlePoints_opt[i]);
                }                

                if (toolIndx_detectneedle == 1)
                {
                    nHp.at<double>(0,3) = NeedlePoints[NeedlePoints.size()-1].x;
                    nHp.at<double>(1,3) = NeedlePoints[NeedlePoints.size()-1].y;
                    nHp.at<double>(2,3) = NeedlePoints[NeedlePoints.size()-1].z;
                    tHp = needlePoseFinal * nHp;
                    tHp.copyTo(ToolLHNeedleTip);
                }
                else
                {
                    nHp.at<double>(0,3) = NeedlePoints[NeedlePoints.size()-1].x;
                    nHp.at<double>(1,3) = NeedlePoints[NeedlePoints.size()-1].y;
                    nHp.at<double>(2,3) = NeedlePoints[NeedlePoints.size()-1].z;
                    tHp = needlePoseFinal * nHp;
                    tHp.copyTo(ToolRHNeedleTip);
                }

                pose3Dto2D->printNeedlePntsGlobalImage(GlobaleImages.frameLeft, GlobaleImages.frameRight, NeedlePoints_cal,
                                                       toolIndx_detectneedle, 0, 255, 0);

                // Debug feature image and energy------------------------------------
                needleEnergyL_mat = pose3Dto2D->energyMat_L;
                needleEnergyR_mat = pose3Dto2D->energyMat_R;

                // save for inspection ----------------------------------------------
//                imwrite(ename_needleEnergyL.c_str(), needleEnergyL_mat);
//                imwrite(ename_needleEnergyR.c_str(), needleEnergyR_mat);
                imwrite(ename_needleEnergyL.c_str(), GlobaleImages.frameLeft);
                imwrite(ename_needleEnergyR.c_str(), GlobaleImages.frameRight);

                needleEnergyPixel_L.clear();
                needleEnergyPixel_R.clear();
                for (int i=0; i<pose3Dto2D->energy_L.size(); i++)
                {
                    needleEnergyPixel_L.push_back(pose3Dto2D->energy_L[i]);
                    needleEnergyPixel_R.push_back(pose3Dto2D->energy_R[i]);
                }

                saveAllNeedleResults(pose3Dto2D->needle3D_all, pose3Dto2D);

                needleDetected = true;
				
//#ifdef SimulationON
//#else
                //if (saveDetecedNeedle)
//                    SaveImage = true;
//#endif
            }
          
        }
*/


        //         Save image -------------------------
        if (SaveImage)
        {
            saveStereoImages2(imageInd);
            SaveImage = false;
            imageInd++;
        }


        GlobaleImages.frameLeft.copyTo(GlobaleImages_show.frameLeft);
        GlobaleImages.frameRight.copyTo(GlobaleImages_show.frameRight);


        emit imagesReady(GlobaleImages_show);



        if (sigREcod==true)
            recordVideo();
		
    }


}

Matrix4d CV2EigenMat(cv::Mat & cvmat)
{
    Matrix4d eigenMat;
    for (int i=0; i<cvmat.rows;i++)
        for (int j=0; j<cvmat.cols;j++ )
        eigenMat(i,j)=cvmat.at<double> (i,j);
    return eigenMat;
}

void VisualTrackingThread::initTools(vector<float> mSizes, vector<vector<int> > tools_ID)
{
    this->numTools = tools_ID.size();
    toolsTracker->initTools(mSizes, tools_ID, filterWindow);

    toolsTvec_hist.resize(numTools);
    toolsRvec_hist.resize(numTools);

    cout << "finish initTools"<<endl;
}

stereoImages VisualTrackingThread::showNeedleDriverFrameInLeft(stereoImages &Images)
{
    Vector4d origin(0,0,0,1);
    Vector4d x(0.025,0,0,1);
    Vector4d y(0,0.025,0,1);
    Vector4d z(0,0,0.025,1);

    Matrix4d  needleDriverPoseInCamEigen = CV2EigenMat(needleDriverPoseInCamerma) ;

    Vector4d originInCam = needleDriverPoseInCamEigen*origin;
    Vector4d xCam = needleDriverPoseInCamEigen*x;
    Vector4d yInCam = needleDriverPoseInCamEigen*y;
    Vector4d zInCam = needleDriverPoseInCamEigen*z;

    cv::Point3d originCV;
    cv::Point3d xCV;
    cv::Point3d yCV;
    cv::Point3d zCV;

    originCV.x = originInCam(0);
    originCV.y = originInCam(1);
    originCV.z = originInCam(2);

    xCV.x = xCam(0);
    xCV.y = xCam(1);
    xCV.z = xCam(2);

    yCV.x = yInCam(0);
    yCV.y = yInCam(1);
    yCV.z = yInCam(2);

    zCV.x = zInCam(0);
    zCV.y = zInCam(1);
    zCV.z = zInCam(2);

    std::vector<cv::Point3d> framePoints3D;
    std::vector<cv::Point2d> framePoints2D;

    framePoints3D.push_back(originCV);
    framePoints3D.push_back(xCV);
    framePoints3D.push_back(yCV);
    framePoints3D.push_back(zCV);


    Mat t_vec = Mat(1,3, CV_64F, double(0));
    Mat r_vec = Mat(1,3, CV_64F, double(0));


    Mat P1_ = P1(cv::Rect(0,0,3,3));
    cv::projectPoints(framePoints3D, t_vec, r_vec, P1_, D1, framePoints2D);

//    cv::Mat frameLeft_rect;
//    frameLeft_rect.create( Images.frameLeft.size(), frameLeft_rect.type() );
//    remap(Images.frameLeft, frameLeft_rect, mx1, my1, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );

    //Drawing Frame
    line(Images.frameLeft, framePoints2D[0], framePoints2D[1], Scalar(0,0,255), 2, 8, 0);
    line(Images.frameLeft, framePoints2D[0], framePoints2D[2], Scalar(0,255,0), 2, 8, 0);
    line(Images.frameLeft, framePoints2D[0], framePoints2D[3], Scalar(255,0,0), 2, 8, 0);

//    frameLeft_rect.copyTo(Images.frameLeft);

    return Images;
}

std::string VisualTrackingThread::initializeRecodeVideo(string fileDir, char *buffer )
{
//    time_t t = time(0);   // get time now
//    struct tm * now = localtime( & t );
//    char buffer [80];
//    strftime (buffer,80,"%Y-%m-%d-%H-%M",now);
    std::string strTempF = "trajectory_" + string(buffer) + ".txt";
    std::string strTemp;

    // write a video
    strTemp = fileDir+"outL_" + string(buffer) + ".avi";
    videoL.open(strTemp.c_str(),CV_FOURCC('M','J','P','G'),10, Size(GlobaleImages.frameLeft.size().width, GlobaleImages.frameLeft.size().height),true);
    strTemp =fileDir+ "outR_" + string(buffer) + ".avi";
    videoR.open(strTemp.c_str(),CV_FOURCC('M','J','P','G'),10, Size(GlobaleImages.frameRight.size().width, GlobaleImages.frameRight.size().height),true);

    if (!videoL.isOpened() || !videoR.isOpened())
    {
        std::cout << "!!! Output video could not be opened" << std::endl;
    }
    cout << "Start recording!"<<endl;
    sigREcod=true;
    return strTempF;
}

void VisualTrackingThread::recordVideo()
{
    videoL.write(GlobaleImages.frameLeft);
    videoR.write(GlobaleImages.frameRight);

//    cv::Mat frameLv, frameRv;
//    camera.Left>>frameLv;
//    camera.Right>> frameRv;
//    videoL.write(frameLv);
//    videoR.write(frameRv);
}

string IntToStr(int n)
{
    stringstream result;
    result << n;
    return result.str();
}

bool VisualTrackingThread::saveStereoImages(int imageRef, string fname_l, string fname_r)
{
    fname_l = (SRC_FILES_DIR+"VisionSystem/caliInfo/left" +  IntToStr(imageRef) + ".jpg");
    fname_r = (SRC_FILES_DIR+"VisionSystem/caliInfo/right" + IntToStr(imageRef) + ".jpg");

    stereoImages imagTemp;
    GlobaleImages.frameLeft.copyTo(imagTemp.frameLeft);
    GlobaleImages.frameRight.copyTo(imagTemp.frameRight);
    vector<cv::Point2f> cornersLeft;
    vector<cv::Point2f> cornersRight;

    int imageCunt=0;

    while (!trackingNeedleDriver->KeyDotsObserved( imagTemp ,cornersLeft,cornersRight))
    {
        imageCunt++;
        if (imageCunt==100)
        {
            cout<<"skip this point"<<endl;
            break;
        }
        msleep(2);
    }
      if (imageCunt<100)
      {
           imwrite(fname_l.c_str(), imagTemp.frameLeft);
           imwrite(fname_r.c_str(), imagTemp.frameRight);
           cout << fname_l << " saved" << endl;
           return true;
      }
      else
      {
        return false;
      }
       //cout<<"keydots detected"<<endl;
}

bool VisualTrackingThread::saveStereoImages2(int imageRef)
{
    cout << "saving image!" << endl;
    timer = time(0);   // get time now
    struct tm * now = localtime( & timer );
    char buffer [80];
    strftime (buffer,80,"%Y-%m-%d-%H-%M-%S-",now);
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds

    string fname_l = ("Data/left_" +  string(buffer) + IntToStr(imageRef) + ".png");
    string fname_r = ("Data/right_" + string(buffer) + IntToStr(imageRef) + ".png");
    string fname_l_raw = ("Data/left_" +  string(buffer) + IntToStr(imageRef) + "_raw.png");
    string fname_r_raw = ("Data/right_" + string(buffer) + IntToStr(imageRef) + "_raw.png");

    stereoImages imageRaw, imagePro;

    GlobaleImages.frameLeft.copyTo(imagePro.frameLeft);
    GlobaleImages.frameRight.copyTo(imagePro.frameRight);
    imwrite(fname_l.c_str(), imagePro.frameLeft);
    imwrite(fname_r.c_str(), imagePro.frameRight);

    if (needleEnergyL_mat.rows == 480)
    {
        string ename_l = ("Data/EngergyL_" +  string(buffer) + IntToStr(imageRef) + ".png");
        string ename_r = ("Data/EngergyR_" +  string(buffer) + IntToStr(imageRef) + ".png");

        imwrite(ename_l.c_str(), needleEnergyL_mat);
        imwrite(ename_r.c_str(), needleEnergyR_mat);

        ofstream f_energy;
        f_energy.open("Data/energyPixel_" +  string(buffer) + IntToStr(imageRef) + ".txt");
        for (int i=0; i<needleEnergyPixel_L.size(); i++)
        {
            f_energy << needleEnergyPixel_L[i] << " " << needleEnergyPixel_R[i] << endl;
        }

        ofstream f_tHn_;
        f_tHn_.open("Data/tHn_" +  string(buffer) + IntToStr(imageRef) + ".txt");
        for (int i=0; i<4; i++)
        {
            for (int j=0; j<4; j++)
                {
                    f_tHn_ << tHn_.at<double>(i,j) << " ";
                }
            f_tHn_ << endl;
        }
    }

    {
        Mat leftImage_gray, rightImage_gray;
        cvtColor(GlobaleImages.frameLeft, leftImage_gray, CV_RGB2GRAY);
        cvtColor(GlobaleImages.frameRight, rightImage_gray, CV_RGB2GRAY);
        IplImage currFrame1cLeft;
        IplImage currFrame1cRight;
        currFrame1cLeft = leftImage_gray;
        currFrame1cRight = rightImage_gray;

//        IplImage *f_left, *f_right;
//        cv::Mat image1;
//        IplImage* image2;
//        image2 = cvCreateImage(cvSize(image1.cols,image1.rows),8,3);
//        IplImage ipltemp=image1;
//        cvCopy(&ipltemp,image2);

        needlePoseEstimation * pose3Dto2D = new needlePoseEstimation();
        D1.copyTo(pose3Dto2D->D1);
        D2.copyTo(pose3Dto2D->D2);
        P1.copyTo(pose3Dto2D->P1);
        P2.copyTo(pose3Dto2D->P2);

        IplImage * energyIPL_L, *energyIPL_R;
        energyIPL_L = cvCreateImage(cvGetSize(&currFrame1cLeft), IPL_DEPTH_64F,1);
        energyIPL_R = cvCreateImage(cvGetSize(&currFrame1cRight), IPL_DEPTH_64F,1);
//        cvCopy(&currFrame1cLeft, energyIPL_L);
//        cvCopy(&currFrame1cRight, energyIPL_R);
        pose3Dto2D->HessianAnalysisEigenvalues(&currFrame1cLeft, 2.0, 13, energyIPL_L);
        pose3Dto2D->HessianAnalysisEigenvalues(&currFrame1cRight, 2.0, 13, energyIPL_R);

        cvConvertScale(energyIPL_L, energyIPL_L, 255, 0); //255
        cvConvertScale(energyIPL_R, energyIPL_R, 255, 0);

//        energyIPL_L = cvCloneImage(currFrame1cLeft);
//        energyIPL_R = cvCloneImage(currFrame1cRight);

        Mat energyMat_L, energyMat_R;
        cv::cvarrToMat(energyIPL_L).copyTo(energyMat_L);
        cv::cvarrToMat(energyIPL_R).copyTo(energyMat_R);

        string ename_l = ("Data/PureEngergyL_" +  string(buffer) + IntToStr(imageRef) + ".png");
        string ename_r = ("Data/PureEngergyR_" +  string(buffer) + IntToStr(imageRef) + ".png");

//        cvSaveImage("foo.png",&currFrame1cLeft);

        imwrite(ename_l.c_str(), energyMat_L);
        imwrite(ename_r.c_str(), energyMat_R);
    }

    cout << fname_l << " saved" << endl;

#ifdef SimulationON
    camera.Left >> imageRaw.frameLeft;
    camera.Right >> imageRaw.frameRight;
    imwrite(fname_l_raw.c_str(), imageRaw.frameLeft);
    imwrite(fname_r_raw.c_str(), imageRaw.frameRight);
#else
    camera.Left >> imageRaw.frameLeft;
    camera.Right >> imageRaw.frameRight;
    imwrite(fname_l_raw.c_str(), imageRaw.frameLeft);
    imwrite(fname_r_raw.c_str(), imageRaw.frameRight);
#endif

    return true;
}



bool VisualTrackingThread::saveChessboardImages(int imageRef, string fname_l, string fname_r)
{
    fname_l = (SRC_FILES_DIR+"VisionSystem/caliInfo/left" +  IntToStr(imageRef) + ".jpg");
    fname_r = (SRC_FILES_DIR+"VisionSystem/caliInfo/right" + IntToStr(imageRef) + ".jpg");

    stereoImages imagTemp;
    GlobaleImages.frameLeft.copyTo(imagTemp.frameLeft);
    GlobaleImages.frameRight.copyTo(imagTemp.frameRight);

    imwrite(fname_l.c_str(), imagTemp.frameLeft);
    imwrite(fname_r.c_str(), imagTemp.frameRight);
}

bool VisualTrackingThread::calculateAndSaveMarkerPoseInCameraFrame()
{
    //count image number
    DIR *dpdf;
    struct dirent *epdf;
    std::vector<string> fileNames;
    const char* imageList=(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliInput/list.txt").c_str();
    ifstream f_stream(imageList);

//    while (!f_stream.eof())
//    {
//        fileNames.push_back( epdf->d_name  );
//                if (f_stream >> xyz)

//    }
    dpdf = opendir(strdup((SRC_FILES_DIR+"VisionSystem/caliInfo").c_str()));
    if (dpdf != NULL){
       while (epdf = readdir(dpdf))
       {
           if ( strstr( epdf->d_name, "left" ))
           {
                //cout<<epdf->d_name<<endl;
                fileNames.push_back( epdf->d_name  );
           }
       }
    }

    ofstream fileMarkInCarmeraFrame;
    fileMarkInCarmeraFrame.open(SRC_FILES_DIR+"VisionSystem/caliInfo/handEyeOutput/markPoseInCameraFrame.txt" );
    //load image pairs
    for (int i=0;i < fileNames.size() ;i++)
    {
        string fname_l = (SRC_FILES_DIR+"VisionSystem/caliInfo/left" +  IntToStr(i) + ".jpg");
        string fname_r = (SRC_FILES_DIR+"VisionSystem/caliInfo/right" + IntToStr(i) + ".jpg");
        f_stream >> fname_l;
        f_stream >> fname_r;
        stereoImages ImagesTemp;
        ImagesTemp.frameLeft=imread (fname_l, CV_LOAD_IMAGE_COLOR);
        ImagesTemp.frameRight=imread(fname_r, CV_LOAD_IMAGE_COLOR);
        cv::Mat needleDriverPoseTemp;
        std::vector<cv::Point2f> pointsLeft;
        std::vector<cv::Point2f> pointsRight;

        //while (! trackingNeedleDriver->findPatternPose( ImagesTemp ,needleDriverPoseTemp) )
        int interCnt=0;
        bool boolWrite2File= true;
        while(!trackingNeedleDriver->KeyDotsObserved(ImagesTemp, pointsLeft, pointsRight))
        {
            interCnt++;
            if (interCnt>100)
            {
                cout<<"skip this point when do hand eye\n"<<endl;
                boolWrite2File=false;
                break;
            }
            msleep(1);
        }

        if( boolWrite2File &&trackingNeedleDriver->findPatternPose(pointsLeft, pointsRight, needleDriverPoseTemp))
        {
                //msleep(1);
                Matrix4d tempMat=Matrix4d::Identity();
                for (int m=0;m<4;m++)
                    for(int n=0;n<4;n++)
                        tempMat(m,n)=needleDriverPoseTemp.at<double>(m,n);

                fileMarkInCarmeraFrame<<tempMat<<endl;
            cout<<" keydot found "<<i<<endl;
        }
        else
        {
            if (!boolWrite2File)
            {
                Matrix4d tempMat=Matrix4d::Ones();
                tempMat=-1000*tempMat;
                fileMarkInCarmeraFrame<< tempMat  <<endl;
            }
        }
    }
    cout<< "keydots in camera frame saved"<<endl;
    return true;
}

bool VisualTrackingThread::getNeedleDriverMarkerPoseInCamFrame(cv::Mat &Pose )
{
    stereoImages ImagesTemp;
    GlobaleImages.frameLeft.copyTo(ImagesTemp.frameLeft);
    GlobaleImages.frameRight.copyTo(ImagesTemp.frameRight);
    std::vector<cv::Point2f> pointsLeft;
    std::vector<cv::Point2f> pointsRight;

    //find ket dots' point in images
    while(!trackingNeedleDriver->KeyDotsObserved(ImagesTemp, pointsLeft, pointsRight))
    {
        msleep(1);
    }
    //triangulation
    if( trackingNeedleDriver->findPatternPose(pointsLeft, pointsRight, Pose))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool VisualTrackingThread::detectToolsInCamFrame(vector<bool> &toolDetected)
{
    // TODO:  copy own images, plot in small window

    cv::Mat frameL, frameR;
    GlobaleImages.frameLeft.copyTo(frameL);
    GlobaleImages.frameRight.copyTo(frameR);

    vector<cv::Mat> toolTvec, toolRvec;
    toolTvec.resize(numTools);
    toolRvec.resize(numTools);

//    toolsTracker->detectTools(GlobaleImages.frameLeft, GlobaleImages.frameRight, toolTvec, toolRvec, toolDetected);
    toolsTracker->detectTools(frameL, frameR, toolTvec, toolRvec, toolDetected);

    if (DrawTool)
    {
        toolsTracker->drawTools(frameL);
        toolsTracker->drawStereoMarkers(frameL, frameR);
    }

    //cv::imshow("left",frameL);

    return true;
}

bool VisualTrackingThread::detectToolsInCamFrame_global(cv::Mat3b frameL, cv::Mat3b frameR)
{
    vector<cv::Mat> toolTvec, toolRvec;
    toolTvec.resize(numTools);
    toolRvec.resize(numTools);

//    toolsTracker->detectTools(GlobaleImages.frameLeft, GlobaleImages.frameRight, toolTvec, toolRvec, toolDetected);
    vector<bool> toolDetected;
    //toolsTracker->detectTools(frameL, frameR, toolTvec, toolRvec, toolDetected);
    toolsTracker->trackTools(frameL, frameR, toolTvec, toolRvec, toolDetected);
    toolsTracker->drawTools(GlobaleImages.frameLeft);
    toolsTracker->drawStereoMarkers(GlobaleImages.frameLeft, GlobaleImages.frameRight);


    DrawNeedle = false;
    if (DrawNeedle)
    {
        toolsTracker->drawNeedle(GlobaleImages.frameLeft, true, ToolLHNeedle, ToolLHNeedleTip, 1);
        toolsTracker->drawNeedle(GlobaleImages.frameRight, false, ToolLHNeedle, ToolLHNeedleTip, 1);
//        toolsTracker->drawNeedle(GlobaleImages.frameLeft, true, ToolRHNeedle, ToolRHNeedleTip, 2);
//        toolsTracker->drawNeedle(GlobaleImages.frameRight, false, ToolRHNeedle, ToolRHNeedleTip, 2);
//        toolsTracker->drawTip(GlobaleImages.frameLeft, true, ToolRHTipL, ToolRHTipR);
//        toolsTracker->drawTip(GlobaleImages.frameRight, false, ToolRHTipL, ToolRHTipR);

        //toolsTracker->drawKalmans(GlobaleImages.frameLeft);
    }
    return true;
}




//this function and above function need modified. they are overlaped
bool VisualTrackingThread::detectMarkerInCamFrame( Matrix4d & MarkerInCamFrame)
{
    stereoImages  ImagesTemp;
    GlobaleImages.frameLeft.copyTo(ImagesTemp.frameLeft);
    GlobaleImages.frameRight.copyTo(ImagesTemp.frameRight);
    cv:Mat MarkerInCamFrameCVFomate;
    std::vector<cv::Point2f> pointsLeft;
    std::vector<cv::Point2f> pointsRight;

    if ( trackingNeedleDriver->KeyDotsObserved(ImagesTemp, pointsLeft, pointsRight))
    {
       trackingNeedleDriver->findPatternPose(pointsLeft, pointsRight, MarkerInCamFrameCVFomate);
       MarkerInCamFrame =  CV2EigenMat( MarkerInCamFrameCVFomate );
       return true;
    }
    else
    {
        return false;
    }
}

//this function and above function need modified. they are overlaped
bool VisualTrackingThread::detectMarkerInCamFrame_rect( Matrix4d & MarkerInCamFrame, bool needRectify)
{

    stereoImages  ImagesTemp;
    GlobaleImages.frameLeft.copyTo(ImagesTemp.frameLeft);
    GlobaleImages.frameRight.copyTo(ImagesTemp.frameRight);

    // Rectify images --------------------
    if (needRectify)
    {
        cv::Mat frameLeft_rect, frameRight_rect;
        frameLeft_rect.create( ImagesTemp.frameLeft.size(), frameLeft_rect.type() );
        remap(ImagesTemp.frameLeft, frameLeft_rect, mx1, my1, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );
        frameRight_rect.create( ImagesTemp.frameRight.size(), frameRight_rect.type() );
        remap(ImagesTemp.frameRight, frameRight_rect, mx2, my2, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );

        ImagesTemp.frameLeft = frameLeft_rect.clone();
        ImagesTemp.frameRight = frameRight_rect.clone();
    }
    //------------------------------------



    cv:Mat MarkerInCamFrameCVFomate;
    std::vector<cv::Point2f> pointsLeft;
    std::vector<cv::Point2f> pointsRight;

    if ( trackingNeedleDriver->KeyDotsObserved(ImagesTemp, pointsLeft, pointsRight))
    {
//       cout << "Points L: " << pointsLeft[0] << endl;
//       cout << "Points R: " << pointsRight[0] << endl;
       if (trackingNeedleDriver->findPatternPose(pointsLeft, pointsRight, MarkerInCamFrameCVFomate))
       {
           MarkerInCamFrame =  CV2EigenMat( MarkerInCamFrameCVFomate );
           return true;
       }
        else
       {
           return false;
       }
    }
    else
    {
        return false;
    }
}


void VisualTrackingThread::drawTrajectory(vector<Point3f> trajectory, int r, int g, int b)
{

    Mat t_vec1 = Mat(3,1, CV_64F, double(0));
    Mat r_vec1 = Mat(3,1, CV_64F, double(0));

    Mat t_vec2 = Mat(3,1, CV_64F, double(0));
    Mat r_vec2 = Mat(3,1, CV_64F, double(0));

//    Mat P1_ = P1(cv::Rect(0,0,3,3));
//    Mat P2_ = P2(cv::Rect(0,0,3,3));

    Mat P1_ = Mat(3,3, CV_64F, double(0));
    Mat P2_ = Mat(3,3, CV_64F, double(0));

    Mat D1_ = Mat(4,1, CV_64F, double(0));
    Mat D2_ = Mat(4,1, CV_64F, double(0));

    cv::Mat T_homogeneous(4,1,cv::DataType<double>::type); // translation vector
    cv::decomposeProjectionMatrix(P1, P1_, r_vec1, T_homogeneous);
    cv::decomposeProjectionMatrix(P2, P2_, r_vec2, T_homogeneous);

    t_vec2.at<double>(0,0) = T_homogeneous.at<double>(0,0)/T_homogeneous.at<double>(3,0);
    t_vec2.at<double>(1,0) = T_homogeneous.at<double>(1,0)/T_homogeneous.at<double>(3,0);
    t_vec2.at<double>(2,0) = T_homogeneous.at<double>(2,0)/T_homogeneous.at<double>(3,0);

    std::vector<cv::Point2f> trajectoryLeft;
    std::vector<cv::Point2f> trajectoryRight;

    cv::projectPoints(trajectory, r_vec1, t_vec1, P1_, D1_, trajectoryLeft);
    cv::projectPoints(trajectory, r_vec2, -t_vec2, P2_, D2_, trajectoryRight);
///////////////////////////////////////////////////////
		 
		if (r==255){
				ofstream check3DTraj("/home/charlie/Documents/workspace/matlab/3DTraj.txt"); 
				for(int j = 0; j <trajectoryLeft.size();j++){
						cv::Mat point4D;
						triangulatePoints(P1, P2, Mat(trajectoryLeft[j]), Mat(trajectoryRight[j]), point4D);
						check3DTraj<<trajectoryLeft[j].x<<" "<<trajectoryLeft[j].y<<" "<<trajectoryRight[j].x<<" "<<trajectoryRight[j].y<<" ";
				
						for (int i = 0; i < 3; i++) { 
								check3DTraj<<point4D.at<float>(0,i)/point4D.at<float>(0,3)<<" ";
						}
						check3DTraj<<endl;

				}
		}
///////////////////////////////////////////////////
    for(int i=0; i<trajectory.size();i++)
    {
        cv::circle(GlobaleImages.frameLeft, trajectoryLeft[i], 0.8, Scalar(r,g,b), -1, 8, 0);
        cv::circle(GlobaleImages.frameRight, trajectoryRight[i], 0.8, Scalar(r,g,b), -1, 8, 0);
    }

}



void VisualTrackingThread::drawNeedleTrajectory(vector<Point3f> toolTraj, vector<Point3f> toolTraj_Rvec, int r, int g, int b)
{
    vector<Point3f> trajectory;
    Marker initPose;

  //  if(bool_draw_tooltip)
    {// Draw the tool tip trajectory
        cv::Mat lHn, cHl, cHn, rvec;
        lHn = Mat::eye(4,4,CV_32F); cHl = Mat::eye(4,4,CV_32F); cHn = Mat::eye(4,4,CV_32F);
        Point3f tmpTip;
        rvec = Mat(3,1,CV_32F);

        ToolLHNeedleTip.copyTo(lHn);
        lHn.convertTo(lHn,CV_32F);

        for (int i=0; i<toolTraj.size(); i++)
        {
            cHl.at<float>(0,3) = toolTraj[i].x;
            cHl.at<float>(1,3) = toolTraj[i].y;
            cHl.at<float>(2,3) = toolTraj[i].z;
            rvec.at<float>(0,0) = toolTraj_Rvec[i].x;
            rvec.at<float>(1,0) = toolTraj_Rvec[i].y;
            rvec.at<float>(2,0) = toolTraj_Rvec[i].z;
            cv::Rodrigues(rvec, cHl.colRange(0,3).rowRange(0,3));

            cHn = cHl * lHn;
            cv::Rodrigues(cHn.colRange(0,3).rowRange(0,3), rvec);

            tmpTip.x = cHn.at<float>(0,3);
            tmpTip.y = cHn.at<float>(1,3);
            tmpTip.z = cHn.at<float>(2,3);
            //Needle.ssize = Tools[1].ssize;
            trajectory.push_back(tmpTip);

            if (i==0)
            {
                initPose.ssize = 0.005;
                initPose.Tvec.ptr<float>(0)[0] = cHn.at<float>(0,3);
                initPose.Tvec.ptr<float>(0)[1] = cHn.at<float>(1,3);
                initPose.Tvec.ptr<float>(0)[2] = cHn.at<float>(2,3);
                initPose.Rvec.ptr<float>(0)[0] = toolTraj_Rvec[i].x;
                initPose.Rvec.ptr<float>(0)[1] = toolTraj_Rvec[i].y;
                initPose.Rvec.ptr<float>(0)[2] = toolTraj_Rvec[i].z;

               // CvDrawingUtils::draw3dAxis(GlobaleImages.frameLeft, initPose, toolsTracker->StereoCameraParametersL);
            }
        }

    }

    Mat t_vec1 = Mat(3,1, CV_64F, double(0));
    Mat r_vec1 = Mat(3,1, CV_64F, double(0));

    Mat t_vec2 = Mat(3,1, CV_64F, double(0));
    Mat r_vec2 = Mat(3,1, CV_64F, double(0));

    Mat D1_ = Mat(4,1, CV_64F, double(0));
    Mat D2_ = Mat(4,1, CV_64F, double(0));

    Mat P1_ = Mat(3,3, CV_64F, double(0));
    Mat P2_ = Mat(3,3, CV_64F, double(0));

    cv::Mat T_homogeneous(4,1,cv::DataType<double>::type); // translation vector
    cv::decomposeProjectionMatrix(P1, P1_, r_vec1, T_homogeneous);
    cv::decomposeProjectionMatrix(P2, P2_, r_vec2, T_homogeneous);

    t_vec2.at<double>(0,0) = T_homogeneous.at<double>(0,0)/T_homogeneous.at<double>(3,0);
    t_vec2.at<double>(1,0) = T_homogeneous.at<double>(1,0)/T_homogeneous.at<double>(3,0);
    t_vec2.at<double>(2,0) = T_homogeneous.at<double>(2,0)/T_homogeneous.at<double>(3,0);

    std::vector<cv::Point2f> trajectoryLeft;
    std::vector<cv::Point2f> trajectoryRight;

    cv::projectPoints(trajectory, r_vec1, t_vec1, P1_, D1_, trajectoryLeft);
    cv::projectPoints(trajectory, r_vec2, -t_vec2, P2_, D2_, trajectoryRight);

    for(int i=0; i<trajectory.size();i++)
    {
        cv::circle(GlobaleImages.frameLeft, trajectoryLeft[i], 0.5, Scalar(r,g,b), -1, 8, 0);
        cv::circle(GlobaleImages.frameRight, trajectoryRight[i], 0.5, Scalar(r,g,b), -1, 8, 0);
    }
}

void VisualTrackingThread::drawThreadForce(double x, double y, double z, int nx, int ny, int nz)
{
    vector<cv::Point3f> v;
    Point3f p0, p1;
    p0.x = x; p0.y = y; p0.z = z;
    v.push_back(p0);

    double norm_f;
    norm_f = 10000;//pow((pow(nx,2)+pow(ny,2)+pow(nz,2)),0.5) * 100;

     if ((pow(nx,2)+pow(ny,2)+pow(nz,2))>0 )
    {
        double sx, sy, sz;
        sx = double(nx)/norm_f; sy = double(ny)/norm_f; sz = double(nz)/norm_f;


        cout << "nx " << nx << " ny " << ny << " nz " << nz << endl;
        cout << "sx " << sx << " sy " << sy << " sz " << sz << " norm_f " << norm_f << endl;

        cv::Mat cHm, mHs, sHp, sHp0, sHp1, cHp, cHs;
        cHm = cv::Mat::eye(4,4,CV_64F);
        sHp = cv::Mat::eye(4,4,CV_64F);
        mHs = cv::Mat::eye(4,4,CV_64F);

        //cHm
        Mat temp_now = Mat(3, 1, CV_64F, double(0));

        cHm.at<double>(0,3) = (double)toolsTracker->Tools[0].Tvec.ptr<float>(0)[0];
        cHm.at<double>(1,3) = (double)toolsTracker->Tools[0].Tvec.ptr<float>(0)[1];
        cHm.at<double>(2,3) = (double)toolsTracker->Tools[0].Tvec.ptr<float>(0)[2];

        temp_now.at<double>(0,0) = (double)toolsTracker->Tools[0].Rvec.ptr<float>(0)[0];
        temp_now.at<double>(1,0) = (double)toolsTracker->Tools[0].Rvec.ptr<float>(0)[1];
        temp_now.at<double>(2,0) = (double)toolsTracker->Tools[0].Rvec.ptr<float>(0)[2];
        cv::Rodrigues(temp_now, cHm.colRange(0,3).rowRange(0,3));
//        cv::Rodrigues(temp_now, cHm);
        cout << "cHm " << cHm << endl;

        //sHp
        sHp.at<double>(0,3) = sx;
        sHp.at<double>(1,3) = sy;
        sHp.at<double>(2,3) = sz;
        cout << "sHp " << sHp << endl;

        //mHs
        mHs.at<double>(0,0) = 0; mHs.at<double>(0,2) = 1;
        mHs.at<double>(2,0) = -1; mHs.at<double>(2,2) = 0;
        mHs.at<double>(0,3) = 0.033;
        cout << "mHs " << mHs << endl;


        //cHp
        cHp = cHm * mHs * sHp;
        cHs = cHm * mHs;
        cout << "cHp " << cHp << endl;
        cout << "cHs " << cHs << endl;

//        p0.x = cHs.at<double>(0,3);
//        p0.y = cHs.at<double>(1,3);
//        p0.z = cHs.at<double>(2,3);

        p1.x = cHp.at<double>(0,3) +p0.x;
        p1.y = cHp.at<double>(1,3) +p0.y;
        p1.z = cHp.at<double>(2,3) +p0.z;

//        p1.x = sx + x;
//        p1.y = sy + y;
//        p1.z = sz + z;


        cout << "p0 x " << p0.x << " y " << p0.y << " z " << p0.z << " norm_f " << norm_f << endl;
        cout << "p1 x " << p1.x << " y " << p1.y << " z " << p1.z << " norm_f " << norm_f << endl;

        drawArrow(p0, p1);

        // Debug: plot sensor frame ----------------------------------------
        Marker tmpMarker;
        tmpMarker.Tvec.ptr<float>(0)[0] = cHs.at<double>(0,3);
        tmpMarker.Tvec.ptr<float>(0)[1] = cHs.at<double>(1,3);
        tmpMarker.Tvec.ptr<float>(0)[2] = cHs.at<double>(2,3);
        cv::Rodrigues(cHs.colRange(0,3).rowRange(0,3), temp_now);
        tmpMarker.Rvec.ptr<float>(0)[0] = temp_now.at<double>(0,0);
        tmpMarker.Rvec.ptr<float>(0)[1] = temp_now.at<double>(1,0);
        tmpMarker.Rvec.ptr<float>(0)[2] = temp_now.at<double>(2,0);
        tmpMarker.ssize = toolsTracker->Tools[0].ssize;

        toolsTracker->Tools.push_back(tmpMarker);
        cout << "sensor   " << tmpMarker.Tvec.ptr<float>(0)[0] << " "
                            << tmpMarker.Tvec.ptr<float>(0)[1] << " "
                            << tmpMarker.Tvec.ptr<float>(0)[2] << " "
                            << tmpMarker.Rvec.ptr<float>(0)[0] << " "
                            << tmpMarker.Rvec.ptr<float>(0)[1] << " "
                            << tmpMarker.Rvec.ptr<float>(0)[2] << " "
                             << endl;

        aruco::CameraParameters StereoCameraParametersL;
        StereoCameraParametersL.readFromXMLFile(CAMERA_CALIB_DIR + "camera_left.yml");
        Mat P1_ = P1(cv::Rect(0,0,3,3));
        P1_.copyTo(StereoCameraParametersL.CameraMatrix);
        D1.copyTo(StereoCameraParametersL.Distorsion);

        CvDrawingUtils::draw3dAxis(GlobaleImages.frameLeft, tmpMarker, StereoCameraParametersL);

        cout << "Mandrel " << toolsTracker->Tools[0].Tvec.ptr<float>(0)[0] << " "
                            << toolsTracker->Tools[0].Tvec.ptr<float>(0)[1] << " "
                            << toolsTracker->Tools[0].Tvec.ptr<float>(0)[2] << " "
                            << toolsTracker->Tools[0].Rvec.ptr<float>(0)[0] << " "
                            << toolsTracker->Tools[0].Rvec.ptr<float>(0)[1] << " "
                            << toolsTracker->Tools[0].Rvec.ptr<float>(0)[2] << " "
                             << endl;
        // ------------------------------------------------------------

    }
}

void VisualTrackingThread::drawArrow(cv::Point3f p0, cv::Point3f p1)
{
    Mat t_vec1 = Mat(3,1, CV_64F, double(0));
    Mat r_vec1 = Mat(3,1, CV_64F, double(0));

    Mat t_vec2 = Mat(3,1, CV_64F, double(0));
    Mat r_vec2 = Mat(3,1, CV_64F, double(0));

    Mat D1_ = Mat(4,1, CV_64F, double(0));
    Mat D2_ = Mat(4,1, CV_64F, double(0));

    Mat P1_ = Mat(3,3, CV_64F, double(0));
    Mat P2_ = Mat(3,3, CV_64F, double(0));

    cv::Mat T_homogeneous(4,1,cv::DataType<double>::type); // translation vector
    cv::decomposeProjectionMatrix(P1, P1_, r_vec1, T_homogeneous);
    cv::decomposeProjectionMatrix(P2, P2_, r_vec2, T_homogeneous);

    t_vec2.at<double>(0,0) = T_homogeneous.at<double>(0,0)/T_homogeneous.at<double>(3,0);
    t_vec2.at<double>(1,0) = T_homogeneous.at<double>(1,0)/T_homogeneous.at<double>(3,0);
    t_vec2.at<double>(2,0) = T_homogeneous.at<double>(2,0)/T_homogeneous.at<double>(3,0);

    std::vector<cv::Point2f> p_Left, p_Right;
    std::vector<cv::Point3f> p;
    p.push_back(p0);
    p.push_back(p1);

    cv::projectPoints(p, r_vec1, t_vec1, P1_, D1_, p_Left);
    cv::projectPoints(p, r_vec2, -t_vec2, P2_, D2_, p_Right);

    cv::arrowedLine(GlobaleImages.frameLeft, p_Left[0], p_Left[1], Scalar(0,0,255,255), 1, CV_AA);
    cv::arrowedLine(GlobaleImages.frameRight, p_Right[0], p_Right[1], Scalar(0,0,255,255), 1, CV_AA);

//    for(int i=0; i<p.size();i++)
//    {
//        cv::circle(GlobaleImages.frameLeft, p_Left[i], 3, Scalar(0,255,255), -1, 8, 0);
//        cv::circle(GlobaleImages.frameRight, p_Right[i], 3, Scalar(0,255,255), -1, 8, 0);
//    }
}


void VisualTrackingThread::drawPoints_LeftRight(vector<cv::Point3f> points, int r, int g, int b)
{
    Mat t_vec1 = Mat(3,1, CV_64F, double(0));
    Mat r_vec1 = Mat(3,1, CV_64F, double(0));

    Mat t_vec2 = Mat(3,1, CV_64F, double(0));
    Mat r_vec2 = Mat(3,1, CV_64F, double(0));

    Mat D1_ = Mat(4,1, CV_64F, double(0));
    Mat D2_ = Mat(4,1, CV_64F, double(0));

    Mat P1_ = Mat(3,3, CV_64F, double(0));
    Mat P2_ = Mat(3,3, CV_64F, double(0));

    cv::Mat T_homogeneous(4,1,cv::DataType<double>::type); // translation vector
    cv::decomposeProjectionMatrix(P1, P1_, r_vec1, T_homogeneous);
    cv::decomposeProjectionMatrix(P2, P2_, r_vec2, T_homogeneous);

    t_vec2.at<double>(0,0) = T_homogeneous.at<double>(0,0)/T_homogeneous.at<double>(3,0);
    t_vec2.at<double>(1,0) = T_homogeneous.at<double>(1,0)/T_homogeneous.at<double>(3,0);
    t_vec2.at<double>(2,0) = T_homogeneous.at<double>(2,0)/T_homogeneous.at<double>(3,0);

    std::vector<cv::Point2f> pointsLeft;
    std::vector<cv::Point2f> pointsRight;

    cv::projectPoints(points, r_vec1, t_vec1, P1_, D1_, pointsLeft);
    cv::projectPoints(points, r_vec2, -t_vec2, P2_, D2_, pointsRight);

    for(int i=0; i<points.size();i++)
    {
        cv::circle(GlobaleImages.frameLeft, pointsLeft[i], 0.5, Scalar(r,g,b), -1, 8, 0);
        cv::circle(GlobaleImages.frameRight, pointsRight[i], 0.5, Scalar(r,g,b), -1, 8, 0);
    }
}



void VisualTrackingThread::drawTipTrajectory(vector<Point3f> toolTraj, vector<Point3f> toolTraj_Rvec, int r, int g, int b)
{
    vector<Point3f> trajectory;
    Marker initPose;

  //  if(bool_draw_tooltip)
    {// Draw the tool tip trajectory
        cv::Mat rHt, cHr, cHt, rvec;
        rHt = Mat::eye(4,4,CV_32F); cHr = Mat::eye(4,4,CV_32F); cHt = Mat::eye(4,4,CV_32F);
        Point3f tmpTip;
        rvec = Mat(3,1,CV_32F);

        // Compute toolTip
        ToolRHTipL.copyTo(rHt);
        rHt.convertTo(rHt, CV_32F);
        //rHt.at<float>(0,3) = 0.03365;

        for (int i=0; i<toolTraj.size(); i++)
        {
            cHr.at<float>(0,3) = toolTraj[i].x;
            cHr.at<float>(1,3) = toolTraj[i].y;
            cHr.at<float>(2,3) = toolTraj[i].z;
            rvec.at<float>(0,0) = toolTraj_Rvec[i].x;
            rvec.at<float>(1,0) = toolTraj_Rvec[i].y;
            rvec.at<float>(2,0) = toolTraj_Rvec[i].z;
            cv::Rodrigues(rvec, cHr.colRange(0,3).rowRange(0,3));
//            if (i==0)
//            cout << "rvec before " << rvec.at<float>(0,0) << " " << rvec.at<float>(1,0) << " " << rvec.at<float>(2,0) << " " ;

            cHt = cHr * rHt;
            cv::Rodrigues(cHt.colRange(0,3).rowRange(0,3), rvec);

//            if (i==0)
//            cout << "after " << rvec.at<float>(0,0) << " " << rvec.at<float>(1,0) << " " << rvec.at<float>(2,0) << endl;

            tmpTip.x = cHt.at<float>(0,3);
            tmpTip.y = cHt.at<float>(1,3);
            tmpTip.z = cHt.at<float>(2,3);
            //Needle.ssize = Tools[1].ssize;
            trajectory.push_back(tmpTip);

            if (i==0)
            {
                initPose.ssize = 0.005;
                initPose.Tvec.ptr<float>(0)[0] = cHt.at<float>(0,3);
                initPose.Tvec.ptr<float>(0)[1] = cHt.at<float>(1,3);
                initPose.Tvec.ptr<float>(0)[2] = cHt.at<float>(2,3);
                initPose.Rvec.ptr<float>(0)[0] = toolTraj_Rvec[i].x;
                initPose.Rvec.ptr<float>(0)[1] = toolTraj_Rvec[i].y;
                initPose.Rvec.ptr<float>(0)[2] = toolTraj_Rvec[i].z;
            }
        }

    }
//    else
//    {
//        // Draw the tool trajectory
//        for (int i=0; i<toolTraj.size(); i++)
//        {
//            trajectory.push_back(toolTraj[i]);
//        }
//        initPose.ssize = 0.005;
//        initPose.Tvec.ptr<float>(0)[0] = toolTraj[0].x;
//        initPose.Tvec.ptr<float>(0)[1] = toolTraj[0].y;
//        initPose.Tvec.ptr<float>(0)[2] = toolTraj[0].z;
//        initPose.Rvec.ptr<float>(0)[0] = toolTraj_Rvec[0].x;
//        initPose.Rvec.ptr<float>(0)[1] = toolTraj_Rvec[0].y;
//        initPose.Rvec.ptr<float>(0)[2] = toolTraj_Rvec[0].z;
//    }


    //CvDrawingUtils::draw3dAxis(GlobaleImages.frameLeft, initPose, toolsTracker->StereoCameraParametersL);

    Mat t_vec1 = Mat(3,1, CV_64F, double(0));
    Mat r_vec1 = Mat(3,1, CV_64F, double(0));

    Mat t_vec2 = Mat(3,1, CV_64F, double(0));
    Mat r_vec2 = Mat(3,1, CV_64F, double(0));

//    Mat P1_ = P1(cv::Rect(0,0,3,3));
//    Mat P2_ = P2(cv::Rect(0,0,3,3));

    Mat D1_ = Mat(4,1, CV_64F, double(0));
    Mat D2_ = Mat(4,1, CV_64F, double(0));

    Mat P1_ = Mat(3,3, CV_64F, double(0));
    Mat P2_ = Mat(3,3, CV_64F, double(0));

    cv::Mat T_homogeneous(4,1,cv::DataType<double>::type); // translation vector
    cv::decomposeProjectionMatrix(P1, P1_, r_vec1, T_homogeneous);
    cv::decomposeProjectionMatrix(P2, P2_, r_vec2, T_homogeneous);

    t_vec2.at<double>(0,0) = T_homogeneous.at<double>(0,0)/T_homogeneous.at<double>(3,0);
    t_vec2.at<double>(1,0) = T_homogeneous.at<double>(1,0)/T_homogeneous.at<double>(3,0);
    t_vec2.at<double>(2,0) = T_homogeneous.at<double>(2,0)/T_homogeneous.at<double>(3,0);

    std::vector<cv::Point2f> trajectoryLeft;
    std::vector<cv::Point2f> trajectoryRight;

    cv::projectPoints(trajectory, r_vec1, t_vec1, P1_, D1_, trajectoryLeft);
    cv::projectPoints(trajectory, r_vec2, -t_vec2, P2_, D2_, trajectoryRight);

    for(int i=0; i<trajectory.size();i++)
    {
        cv::circle(GlobaleImages.frameLeft, trajectoryLeft[i], 0.5, Scalar(r,g,b), -1, 8, 0);
        cv::circle(GlobaleImages.frameRight, trajectoryRight[i], 0.5, Scalar(r,g,b), -1, 8, 0);
    }
}




////bool VisualTrackingThread::StereoCalib()
//{
//    const char* imageList=SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliInput/list.txt";
//    int nx=4;
//    int ny=11;
//    int useUncalibrated=0;
//    float _squareSize= 9.676/2.;

//    int displayCorners = 1;
//    int showUndistorted = 1;
//    bool isVerticalStereo = false;//OpenCV can handle left-right
//                                      //or up-down camera arrangements
//    const int maxScale = 1;
//    const float squareSize = _squareSize; //Chessboard square size in cm
//    FILE* f = fopen(imageList, "rt");
//    int i, j, lr, nframes, n = nx*ny, N = 0;
//    vector<string> imageNames[2];
//    vector<CvPoint3D32f> objectPoints;
//    vector<CvPoint2D32f> points[2];
//    vector<int> npoints;
//    vector<uchar> active[2];
//    vector<CvPoint2D32f> temp(n);
//    CvSize imageSize = {0,0};
//    // ARRAY AND VECTOR STORAGE:
//    double M1[3][3], M2[3][3], D1[5], D2[5];
//    double R[3][3], T[3], E[3][3], F[3][3];
//    double Q[4][4];
//    CvMat _M1 = cvMat(3, 3, CV_64F, M1 );
//    CvMat _M2 = cvMat(3, 3, CV_64F, M2 );
//    CvMat _D1 = cvMat(1, 5, CV_64F, D1 );
//    CvMat _D2 = cvMat(1, 5, CV_64F, D2 );
//    CvMat _R = cvMat(3, 3, CV_64F, R );
//    CvMat _T = cvMat(3, 1, CV_64F, T );
//    CvMat _E = cvMat(3, 3, CV_64F, E );
//    CvMat _F = cvMat(3, 3, CV_64F, F );
//    CvMat _Q = cvMat(4,4, CV_64F, Q);
////    if( displayCorners )
////        cvNamedWindow( "corners", 1 );
//// READ IN THE LIST OF CHESSBOARDS:
//    if( !f )
//    {
//        fprintf(stderr, "can not open file %s\n", imageList );
//    }
//    for(i=0;;i++)
//    {
//        char buf[1024];
//        int count = 0, result=0;
//        lr = i % 2;
//        vector<CvPoint2D32f>& pts = points[lr];
//        if( !fgets( buf, sizeof(buf)-3, f ))
//            break;
//        size_t len = strlen(buf);
//        while( len > 0 && isspace(buf[len-1]))
//            buf[--len] = '\0';
//        if( buf[0] == '#')
//            continue;
//        IplImage* img = cvLoadImage( buf, 1 );
//        if( !img )
//            break;
//        imageSize = cvGetSize(img);
//        imageNames[lr].push_back(buf);
//    //FIND CHESSBOARDS AND CORNERS THEREIN:

//        cv::SimpleBlobDetector::Params params;
//        params.minRepeatability = 2;
//        params.minDistBetweenBlobs = 2;
//        params.thresholdStep = 50;
//        params.minArea = 5;
//        params.maxArea = 300;
//        params.minThreshold = 10;
//        params.maxThreshold = 220;
//        params.blobColor = 0 ;


//        cv::Ptr<cv::SimpleBlobDetector> blobDetector = new cv::SimpleBlobDetector(params);

//        cv::Mat frame = cv::cvarrToMat(img);
//        cv::Size pattern_size(nx, ny);
//        cv::Mat gray;
//        //cout<<"cuck"<<std::endl;
//        cv::cvtColor(frame, gray, CV_BGR2GRAY);
//        std::vector<cv::Point> corners;
//        result = cv::findCirclesGrid(gray, pattern_size, corners,
//            cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetector);


//        for (int i = 0; i < corners.size(); i++)
//        {
//            temp[i] = corners[i];
//        }

//        if( displayCorners )
//        {
//            //printf("%s\n", buf);

//            for (int i = 0; i < corners.size(); i++){
//                cv::circle(frame, corners[i], 3, CV_RGB(0, 255 ,0));
//            }
////            cv::imshow("corners", frame);
////            if( cvWaitKey(0) == 27 ) //Allow ESC to quit
////                exit(-1);
//        }
//        else
//            putchar('.');
//        N = pts.size();
//        pts.resize(N + n, cvPoint2D32f(0,0));
//        active[lr].push_back((uchar)result);
//        copy( temp.begin(), temp.end(), pts.begin() + N );
//        cvReleaseImage( &img );
//    }
//    fclose(f);
//   // printf("\n");
//// HARVEST CHESSBOARD 3D OBJECT POINT LIST:
//    nframes = active[0].size();//Number of good chessboads found
//    objectPoints.resize(nframes*n);
//    for( i = 0; i < ny; i++ )
//        for( j = 0; j < nx; j++ )
//            objectPoints[i*nx + j] = cvPoint3D32f((2*j + i%2)*squareSize, i*squareSize, 0);
//    for( i = 1; i < nframes; i++ )
//        copy( objectPoints.begin(), objectPoints.begin() + n,
//        objectPoints.begin() + i*n );
//    npoints.resize(nframes,n);
//    N = nframes*n;
//    CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0] );
//    CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
//    CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
//    CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0] );
//    cvSetIdentity(&_M1);
//    cvSetIdentity(&_M2);
//    cvZero(&_D1);
//    cvZero(&_D2);

//// CALIBRATE THE STEREO CAMERAS
//    cout<<"Running stereo calibration ..."<<endl;
//    fflush(stdout);
//    cvStereoCalibrate( &_objectPoints, &_imagePoints1,
//        &_imagePoints2, &_npoints,
//        &_M1, &_D1, &_M2, &_D2,
//        imageSize, &_R, &_T, &_E, &_F,
//        cvTermCriteria(CV_TERMCRIT_ITER+
//        CV_TERMCRIT_EPS, 100, 1e-5),
//        CV_CALIB_FIX_ASPECT_RATIO +
//        CV_CALIB_ZERO_TANGENT_DIST +
//        CV_CALIB_SAME_FOCAL_LENGTH );
//   //cout<<" done\n"<<endl;
//// CALIBRATION QUALITY CHECK
//// because the output fundamental matrix implicitly
//// includes all the output information,
//// we can check the quality of calibration using the
//// epipolar geometry constraint: m2^t*F*m1=0
//    vector<CvPoint3D32f> lines[2];
//    points[0].resize(N);
//    points[1].resize(N);
//    _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
//    _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
//    lines[0].resize(N);
//    lines[1].resize(N);
//    CvMat _L1 = cvMat(1, N, CV_32FC3, &lines[0][0]);
//    CvMat _L2 = cvMat(1, N, CV_32FC3, &lines[1][0]);
////Always work in undistorted space
//    cvUndistortPoints( &_imagePoints1, &_imagePoints1,
//        &_M1, &_D1, 0, &_M1 );
//    cvUndistortPoints( &_imagePoints2, &_imagePoints2,
//        &_M2, &_D2, 0, &_M2 );
//    cvComputeCorrespondEpilines( &_imagePoints1, 1, &_F, &_L1 );
//    cvComputeCorrespondEpilines( &_imagePoints2, 2, &_F, &_L2 );
//    double avgErr = 0;
//    for( i = 0; i < N; i++ )
//    {
//        double err = fabs(points[0][i].x*lines[1][i].x +
//            points[0][i].y*lines[1][i].y + lines[1][i].z)
//            + fabs(points[1][i].x*lines[0][i].x +
//            points[1][i].y*lines[0][i].y + lines[0][i].z);
//        avgErr += err;
//    }
//    cout<< "avg err =" << avgErr/(nframes*n) <<endl;
////COMPUTE AND DISPLAY RECTIFICATION
//    if( showUndistorted )
//    {
//        CvMat* mx1 = cvCreateMat( imageSize.height,
//            imageSize.width, CV_32F );
//        CvMat* my1 = cvCreateMat( imageSize.height,
//            imageSize.width, CV_32F );
//        CvMat* mx2 = cvCreateMat( imageSize.height,

//            imageSize.width, CV_32F );
//        CvMat* my2 = cvCreateMat( imageSize.height,
//            imageSize.width, CV_32F );
//        CvMat* img1r = cvCreateMat( imageSize.height,
//            imageSize.width, CV_8U );
//        CvMat* img2r = cvCreateMat( imageSize.height,
//            imageSize.width, CV_8U );
//        CvMat* disp = cvCreateMat( imageSize.height,
//            imageSize.width, CV_16S );
//        CvMat* vdisp = cvCreateMat( imageSize.height,
//            imageSize.width, CV_8U );
//        CvMat* pair;
//        double R1[3][3], R2[3][3], P1[3][4], P2[3][4];
//        CvMat _R1 = cvMat(3, 3, CV_64F, R1);
//        CvMat _R2 = cvMat(3, 3, CV_64F, R2);
//// IF BY CALIBRATED (BOUGUET'S METHOD)
//        if( useUncalibrated == 0 )
//        {
//            CvMat _P1 = cvMat(3, 4, CV_64F, P1);
//            CvMat _P2 = cvMat(3, 4, CV_64F, P2);
//            cvStereoRectify( &_M1, &_M2, &_D1, &_D2, imageSize,
//                &_R, &_T,
//                &_R1, &_R2, &_P1, &_P2, &_Q,
//                0/*CV_CALIB_ZERO_DISPARITY*/ );
//            isVerticalStereo = fabs(P2[1][3]) > fabs(P2[0][3]);
//    //Precompute maps for cvRemap()
//            cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_P1,mx1,my1);
//            cvInitUndistortRectifyMap(&_M2,&_D2,&_R2,&_P2,mx2,my2);
//    //Save parameters
//            cvSave(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/M1.xml",&_M1);
//            cvSave(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/D1.xml",&_D1);
//            cvSave(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/R1.xml",&_R1);
//            cvSave(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/P1.xml",&_P1);
//            cvSave(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/M2.xml",&_M2);
//            cvSave(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/D2.xml",&_D2);
//            cvSave(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/R2.xml",&_R2);
//            cvSave(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/P2.xml",&_P2);
//            cvSave(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/Q.xml",&_Q);
//            cvSave(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/mx1.xml",mx1);
//            cvSave(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/my1.xml",my1);
//            cvSave(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/mx2.xml",mx2);
//            cvSave(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/my2.xml",my2);
//            cvSave(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/T.xml", &_T);
//            cvSave(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/R.xml", &_R);
//            cvSave(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/F.xml", &_F);
//        }
////OR ELSE HARTLEY'S METHOD
//        else if( useUncalibrated == 1 || useUncalibrated == 2 )
//     // use intrinsic parameters of each camera, but
//     // compute the rectification transformation directly
//     // from the fundamental matrix
//        {
//            double H1[3][3], H2[3][3], iM[3][3];
//            CvMat _H1 = cvMat(3, 3, CV_64F, H1);
//            CvMat _H2 = cvMat(3, 3, CV_64F, H2);
//            CvMat _iM = cvMat(3, 3, CV_64F, iM);
//    //Just to show you could have independently used F
//            if( useUncalibrated == 2 )
//                cvFindFundamentalMat( &_imagePoints1,
//                &_imagePoints2, &_F);
//            cvStereoRectifyUncalibrated( &_imagePoints1,
//                &_imagePoints2, &_F,
//                imageSize,
//                &_H1, &_H2, 3);
//            cvInvert(&_M1, &_iM);
//            cvMatMul(&_H1, &_M1, &_R1);
//            cvMatMul(&_iM, &_R1, &_R1);
//            cvInvert(&_M2, &_iM);
//            cvMatMul(&_H2, &_M2, &_R2);
//            cvMatMul(&_iM, &_R2, &_R2);
//    //Precompute map for cvRemap()
//            cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_M1,mx1,my1);

//            cvInitUndistortRectifyMap(&_M2,&_D1,&_R2,&_M2,mx2,my2);
//        }
//        else
//            assert(0);
//        //cvNamedWindow( "rectified", 1 );
//// RECTIFY THE IMAGES AND FIND DISPARITY MAPS
//        if( !isVerticalStereo )
//            pair = cvCreateMat( imageSize.height, imageSize.width*2,
//            CV_8UC3 );
//        else
//            pair = cvCreateMat( imageSize.height*2, imageSize.width,
//            CV_8UC3 );
////Setup for finding stereo corrrespondences
//        CvStereoBMState *BMState = cvCreateStereoBMState();
//        assert(BMState != 0);
//        BMState->preFilterSize=41;
//        BMState->preFilterCap=31;
//        BMState->SADWindowSize=41;
//        BMState->minDisparity=-64;
//        BMState->numberOfDisparities=128;
//        BMState->textureThreshold=10;
//        BMState->uniquenessRatio=15;
//        for( i = 0; i < nframes; i++ )
//        {
//            IplImage* img1=cvLoadImage(imageNames[0][i].c_str(),0);
//            IplImage* img2=cvLoadImage(imageNames[1][i].c_str(),0);
//            if( img1 && img2 )
//            {
//                CvMat part;
//                cvRemap( img1, img1r, mx1, my1 );
//                cvRemap( img2, img2r, mx2, my2 );
//                if( !isVerticalStereo || useUncalibrated != 0 )
//                {
//              // When the stereo camera is oriented vertically,
//              // useUncalibrated==0 does not transpose the
//              // image, so the epipolar lines in the rectified
//              // images are vertical. Stereo correspondence
//              // function does not support such a case.
//                    cvFindStereoCorrespondenceBM( img1r, img2r, disp,
//                        BMState);
//                    cvNormalize( disp, vdisp, 0, 256, CV_MINMAX );
////                    cvNamedWindow( "disparity" );
////                    cvShowImage( "disparity", vdisp );
//                }
//                if( !isVerticalStereo )
//                {
//                    cvGetCols( pair, &part, 0, imageSize.width );
//                    cvCvtColor( img1r, &part, CV_GRAY2BGR );
//                    cvGetCols( pair, &part, imageSize.width,
//                        imageSize.width*2 );
//                    cvCvtColor( img2r, &part, CV_GRAY2BGR );
//                    for( j = 0; j < imageSize.height; j += 16 )
//                        cvLine( pair, cvPoint(0,j),
//                        cvPoint(imageSize.width*2,j),
//                        CV_RGB(0,255,0));
//                }
//                else
//                {
//                    cvGetRows( pair, &part, 0, imageSize.height );
//                    cvCvtColor( img1r, &part, CV_GRAY2BGR );
//                    cvGetRows( pair, &part, imageSize.height,
//                        imageSize.height*2 );
//                    cvCvtColor( img2r, &part, CV_GRAY2BGR );
//                    for( j = 0; j < imageSize.width; j += 16 )
//                        cvLine( pair, cvPoint(j,0),
//                        cvPoint(j,imageSize.height*2),
//                        CV_RGB(0,255,0));
//                }
////                cvShowImage( "rectified", pair );
////                if( cvWaitKey() == 27 )
////                    break;
//            }
//            cvReleaseImage( &img1 );
//            cvReleaseImage( &img2 );
//        }
//        cvReleaseStereoBMState(&BMState);
//        cvReleaseMat( &mx1 );
//        cvReleaseMat( &my1 );
//        cvReleaseMat( &mx2 );
//        cvReleaseMat( &my2 );
//        cvReleaseMat( &img1r );
//        cvReleaseMat( &img2r );
//        cvReleaseMat( &disp );
//        cout<<"stereo calibration done !\n"<<endl;
//        return true;
//    }
//}

bool VisualTrackingThread::handeyeCali_abs(char* fname_marker2robot, char *fname_marker2camera, Mat &R_C)
{
    vector<Point3d> m_R, m_C;
    R_C = cv::Mat::eye(4,4,CV_64FC1);
    //cout<< R_C<<endl;
    readPosition(fname_marker2robot, m_R);
    readPosition(fname_marker2camera, m_C);
    pointMatch_abs(m_R, m_C, R_C);
    //pointMatch_rigid(m_R, m_C, R_C);

    ofstream robotInCameraFile;
    robotInCameraFile.open(SRC_FILES_DIR+"VisionSystem/caliInfo/handEyeOutput/robot2CameraTansFile.txt" );

    Matrix4d matTemp  =  CV2EigenMat( R_C  );
    robotInCameraFile<<matTemp<<endl;
    return true;
}

bool VisualTrackingThread::readPosition(char* fname, vector<Point3d> &points)
{
    ifstream f_stream(fname);
    string  xyz;
    cv::Point3d tmpPoint;
    while (!f_stream.eof())
    {
        msleep(1);
        for (int i=0; i<4; i++)
        {
            for (int j=0; j<4; j++)
            {
                if (f_stream >> xyz)
                {
                    if (j==3)
                    {
                        switch (i)
                        {
                            case 0: tmpPoint.x = atof(xyz.c_str()); break;
                            case 1: tmpPoint.y = atof(xyz.c_str()); break;
                            case 2: tmpPoint.z = atof(xyz.c_str());
                                    points.push_back(tmpPoint);
                                    break;
                        }
                    }
                }
            }
        }
    }
    return true;
}

bool VisualTrackingThread::pointMatch_rigid(std::vector<Point3d> &m_R, std::vector<Point3d> &m_C, cv::Mat &R_C)
{
    cout << "hrere"<<endl;
    find_rigidTransformation(m_R,m_C,R_C);
    return true;
}

bool VisualTrackingThread::pointMatch_abs(vector<Point3d> &m_R, vector<Point3d> &m_C, Mat &R_C)
{
    std::vector<Point3D> left(m_R.size()), right(m_C.size());
    for (int i=0; i<m_C.size(); i++)
    {
        if (  !(m_C[i].x<0.0 && m_C[i].y<0.0  && m_C[i].z<0.0) )
        {
            left[i][0] = m_R[i].x;
            left[i][1] = m_R[i].y;
            left[i][2] = m_R[i].z;

            right[i][0] = m_C[i].x;
            right[i][1] = m_C[i].y;
            right[i][2] = m_C[i].z;
        }
    }

    Frame computedTransformation;
    AbsoluteOrientation::compute(left,right,computedTransformation);

    computedTransformation.getRotationMatrix(R_C.at<double>(0,0), R_C.at<double>(0,1), R_C.at<double>(0,2),
                                             R_C.at<double>(1,0), R_C.at<double>(1,1), R_C.at<double>(1,2),
                                             R_C.at<double>(2,0), R_C.at<double>(2,1), R_C.at<double>(2,2));
    computedTransformation.getTranslation(R_C.at<double>(0,3),R_C.at<double>(1,3),R_C.at<double>(2,3));
//
    //cout << R_C <<endl;
    return true;
}


bool VisualTrackingThread::getRelaPoseNeedleAndNeedleDriver(cv::Mat &EEInBase, Matrix4d &newNeedlePose )
{
    cv::Mat leftIamge ;
    GlobaleImages.frameRight.copyTo(leftIamge);
    cv::Mat rightImage;
    GlobaleImages.frameRight.copyTo(rightImage);

    //read needle in end-effector frame
    ifstream f_stream(SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/NeedlePoseInEE_ini.txt");
    cv::Mat needlePoseInEE =Mat(4,4, CV_64F);
    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            {
                double variable;
                f_stream >> variable;
                needlePoseInEE.at<double>(i,j)=variable;
            }
    //cout << "1 needlePoseInEE:\n "<<needlePoseInEE<<endl;

    ifstream f_stream2(SRC_FILES_DIR+"VisionSystem/caliInfo/handEyeOutput/robot2CameraTansFile.txt");
    cv::Mat robotInCamera=Mat(4,4, CV_64F);
    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            {
                double variable;
                f_stream2 >> variable;
                robotInCamera.at<double>(i,j)=variable;
            }
 //   cout << "1 robotInCamera:\n "<<robotInCamera<<endl;

    //needle in camera frame,
    cv::Mat NeedleInCamera = Mat(4,4, CV_64F);

    NeedleInCamera = robotInCamera * EEInBase * needlePoseInEE;

    ifstream f_stream3(SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/NeedlePoseInEE_ini.txt");
    cv::Mat needleDriverInEE=Mat(4,4, CV_64F);
    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            {
                double variable;
                f_stream3 >> variable;
                needleDriverInEE.at<double>(i,j)=variable;
            }
    //cout << "1 needleDriverInEE:\n "<<needleDriverInEE<<endl;

    // needle driver in camera frame
    cv::Mat needleDriverPoseInCam = Mat(4,4, CV_64F);
    needleDriverPoseInCam = robotInCamera* EEInBase *needleDriverInEE;

    cout << "robotInCamera:\n "<<robotInCamera<<endl;
    cout << "EEInBase:\n "<<EEInBase<<endl;
    cout << "needleDriverInEE:\n "<<needleDriverInEE<<endl;


   ifstream f_stream4(SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/MarkerPoseInRobotEndEffector.txt");
   // ifstream f_stream4(SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/ChessBoardInEE.txt");

    //cout<<"here"<<endl;
    cv::Mat markerInEE=Mat(4,4, CV_64F);
    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            {
                double variable;
                f_stream4 >> variable;
                markerInEE.at<double>(i,j)=variable;
            }
    cv::Mat markerPoseInCam = Mat(4,4, CV_64F);
    markerPoseInCam=robotInCamera* EEInBase* markerInEE;

    cv::Point3d markerInCameraXYZ;
    markerInCameraXYZ.x = markerPoseInCam.at<double>(0,3);
    markerInCameraXYZ.y = markerPoseInCam.at<double>(1,3);
    markerInCameraXYZ.z = markerPoseInCam.at<double>(2,3);

//    cout << "markerPoseInCam " << markerPoseInCam << endl;
    bool newHandEye = 1;
//      newHandEye = getMarkerInCameraFromHandEye(EEInBase, markerInCameraXYZ);
//        if (newHandEye)
//        {
//            cout << "markerInCameraXYZ " << markerInCameraXYZ << endl;
//            markerPoseInCam.at<double>(0,3) = markerInCameraXYZ.x;
//            markerPoseInCam.at<double>(1,3) = markerInCameraXYZ.y;
//            markerPoseInCam.at<double>(2,3) = markerInCameraXYZ.z;
//        }

    //showNeedleDriverFrameInCam(needleDriverPoseInCam, newHandEye);
    //showNeedleDriverFrameInCam(needleDriverPoseInCam);

    //from the model
    std::vector<cv::Point3d> needle3Dmodel;

    //test();

    Mat mx1, my1, mx2, my2;

    showNeedleInCam( needleDriverPoseInCam, needle3Dmodel );

    //cvWaitKey();

    FileStorage f_mx1(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/mx1.xml", FileStorage::READ); f_mx1["mx1"]>>mx1;
    FileStorage f_my1(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/my1.xml", FileStorage::READ); f_my1["my1"]>>my1;

    FileStorage f_mx2(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/mx2.xml", FileStorage::READ); f_mx2["mx2"]>>mx2;
    FileStorage f_my2(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/my2.xml", FileStorage::READ); f_my2["my2"]>>my2;

    stereoImages Images;
    GlobaleImages.frameLeft.copyTo(Images.frameLeft);
    GlobaleImages.frameRight.copyTo(Images.frameRight);

    imwrite("Data/original_left.png",Images.frameLeft);
    imwrite("Data/original_right.png",Images.frameRight);

    cv::Mat frameLeft_rect, frameRight_rect;

    frameLeft_rect.create( Images.frameLeft.size(), frameLeft_rect.type() );
    frameRight_rect.create( Images.frameRight.size(), frameRight_rect.type() );

    remap(Images.frameLeft, frameLeft_rect, mx1, my1, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );
    remap(Images.frameRight, frameRight_rect, mx2, my2, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );

    cv::Mat needleFinalRelaInitInCamera=Mat(4,4, CV_64F);
    needleTracking->doPoseEstimation(frameLeft_rect, frameRight_rect, NeedleInCamera, needle3Dmodel, needleDriverPoseInCam, needleFinalRelaInitInCamera);

    needleFinalRelaInitInCamera.at<double>(0,3)=needleFinalRelaInitInCamera.at<double>(0,3)/1000.;
    needleFinalRelaInitInCamera.at<double>(1,3)=needleFinalRelaInitInCamera.at<double>(1,3)/1000.;
    needleFinalRelaInitInCamera.at<double>(2,3)=needleFinalRelaInitInCamera.at<double>(2,3)/1000.;

    cv::Mat  newNeedlePoseCV=Mat(4,4, CV_64F);
    newNeedlePoseCV= EEInBase.inv() * robotInCamera.inv() *( needleFinalRelaInitInCamera) * needleDriverPoseInCam;

    cout << "EEInBase " << EEInBase << endl;
    cout << "robotInCamera "<< robotInCamera << endl;
    cout << "needleDriverPoseInCam " << needleDriverPoseInCam << endl;
    cout << "needleFinalRelaInitInCamera " << needleFinalRelaInitInCamera<< endl;
    cout << "newNeedlePoseCV " << newNeedlePoseCV <<endl;

    ofstream myfile1, myfile2, myfile3, myfile4, myfile5;
    std::stringstream ss1, ss2, ss3, ss4, ss5;
    std::string imageName1, imageName2, imageName3, imageName4, imageName5;

    ss1 << "Data/finalNeedlePose_Robot.txt";
    ss2 << "Data/idealNeedlePose_Robot.txt";
    ss3 << "Data/needleDriverPoseInCam.txt";
    ss4 << "Data/robotInCamera.txt";
    ss5 << "Data/EEInBase.txt";

    imageName1 = ss1.str();
    imageName2 = ss2.str();
    imageName3 = ss3.str();
    imageName4 = ss4.str();
    imageName5 = ss5.str();

    const char * imageNameChar1 = imageName1.c_str();
    const char * imageNameChar2 = imageName2.c_str();
    const char * imageNameChar3 = imageName3.c_str();
    const char * imageNameChar4 = imageName4.c_str();
    const char * imageNameChar5 = imageName5.c_str();

    myfile1.open(imageNameChar1);
    myfile2.open(imageNameChar2);
    myfile3.open(imageNameChar3);
    myfile4.open(imageNameChar4);
    myfile5.open(imageNameChar5);

    myfile1 << newNeedlePoseCV << endl;
    myfile2 << needleDriverInEE << endl;
    myfile3 << needleDriverPoseInCam << endl;
    myfile4 << robotInCamera << endl;
    myfile5 << EEInBase << endl;

    myfile1.close();
    myfile2.close();
    myfile3.close();
    myfile4.close();
    myfile5.close();

    newNeedlePose=CV2EigenMat( newNeedlePoseCV );
    return true;
}


bool VisualTrackingThread::getMarkerInCameraFromHandEye(cv::Mat &EE2R, cv::Point3d &M2C_handeye)
{
    cv::Point3d origin2R = grid_robot[0][0][0];
        int ind_1, ind_2, ind_3;

        ifstream f_cboard2EE(SRC_FILES_DIR+"VisionSystem/caliInfo/transformations/MarkerPoseInRobotEndEffector.txt");
        cv::Mat cboard2EE=Mat(4,4, CV_64F);
        string tmp;
        for (int i=0; i<4; i++)
            for (int j=0; j<4; j++)
                {
                    f_cboard2EE >> tmp;
                    cboard2EE.at<double>(i,j)=atof(tmp.c_str());
                }


        // chess board in robot frame
        cv::Mat cboard2R = Mat(4,4,CV_64FC1);
        cboard2R = EE2R * cboard2EE;

        cout << "cboard2EE "<<cboard2EE<<endl;
        cout << "EE2R "<<EE2R<<endl;
        cout << "cboard2R " << cboard2R <<endl;

        cv::Point3d pt_robot;
        pt_robot.x = cboard2R.at<double>(0,3);
        pt_robot.y = cboard2R.at<double>(1,3);
        pt_robot.z = cboard2R.at<double>(2,3);

        ind_1 = floor(-1* (pt_robot.x - origin2R.x) / GRID_SIZE) + 1;
        ind_2 = floor(-1* (pt_robot.z - origin2R.z) / GRID_SIZE) + 1;
        ind_3 = floor( (pt_robot.y - origin2R.y) / GRID_SIZE) + 1;


        if (ind_1 >= GRID_1 || ind_2 >= GRID_2 || ind_3 >= GRID_3 ||
            ind_1 <= 0 || ind_2 <= 0 || ind_3 <= 0) //Outside grid
            return false;

        // 3D interpolation
        cv::Point3d refO_robot = grid_robot[ind_1][ind_2][ind_3];
        cv::Point3d shift = pt_robot - refO_robot;

        int ind_1s, ind_2s, ind_3s;
        if (shift.x < 0)
            ind_1s = ind_1 + 1;
        else
            ind_1s = ind_1 - 1;
        if (shift.z < 0)
            ind_3s = ind_3 + 1;
        else
            ind_3s = ind_3 - 1;
        if (shift.y > 0)
            ind_2s = ind_2 + 1;
        else
            ind_2s = ind_2 - 1;

        cv::Point3d refX_robot = grid_robot[ind_1s][ind_2][ind_3];
        cv::Point3d refY_robot = grid_robot[ind_1][ind_2][ind_3s];
        cv::Point3d refZ_robot = grid_robot[ind_1][ind_2s][ind_3];

        cv::Point3d refO_camera = grid_camera[ind_1][ind_2][ind_3];

        cv::Point3d refX_camera = grid_camera[ind_1s][ind_2][ind_3];
        cv::Point3d refY_camera = grid_camera[ind_1][ind_2][ind_3s];
        cv::Point3d refZ_camera = grid_camera[ind_1][ind_2s][ind_3];

        cv::Point3d tmp1, tmp2, tmp3;
        tmp1 = shift.x / (refX_robot.x - refO_robot.x) * (refX_camera - refO_camera);
        tmp2 = shift.y / (refY_robot.y - refO_robot.y) * (refY_camera - refO_camera);
        tmp3 = shift.z / (refZ_robot.z - refO_robot.z) * (refZ_camera - refO_camera);

        M2C_handeye = tmp1 + tmp2 + tmp3 + refO_camera;
        return true;
}

void VisualTrackingThread::showNeedleInCam( cv::Mat &needlePoseInCamCV, std::vector<cv::Point3d> &needle3Dmodel )
{
    /*Vector4d origin ( 0,              0, 0,1);
    Vector4d second (-0,         -0.004, 0,1);
    Vector4d middle (-0.006945,  -0.008, 0,1);
    Vector4d third  (-0.0105,    -0.004, 0,1);
    Vector4d tip    (-0.01389,        0, 0,1);*/

   /*Vector4d origin ( 0,            0, 0,1);
    Vector4d second (-0.0015,  -0.0053, 0,1);
    Vector4d middle (-0.006945,-0.008, 0,1);
    Vector4d third  (-0.01239,  -0.0053, 0,1);
    Vector4d tip    (-0.01389,      0, 0,1);*/

    Vector4d origin ( 0,            0, 0,1);
    Vector4d second (-0.00383,  -0.00520, 0,1);
    Vector4d middle (-0.00811,  -0.00658, 0,1);
    Vector4d third  (-0.01314,  -0.00414, 0,1);
    Vector4d tip    (-0.01515,      0, 0,1);

    Matrix4d  needlePoseInCamEigen = CV2EigenMat( needlePoseInCamCV ) ;

    Vector4d originInCam = needlePoseInCamEigen*origin;
    Vector4d secondInCam = needlePoseInCamEigen*second;
    Vector4d middleCam = needlePoseInCamEigen*middle;
    Vector4d thirdInCam = needlePoseInCamEigen*third;
    Vector4d tipInCam = needlePoseInCamEigen*tip;

    cv::Point3d originCV;
    cv::Point3d secondCV;
    cv::Point3d middleCV;
    cv::Point3d thirdCV;
    cv::Point3d tipCV;

    originCV.x = originInCam(0);
    originCV.y = originInCam(1);
    originCV.z = originInCam(2);

    secondCV.x = secondInCam(0);
    secondCV.y = secondInCam(1);
    secondCV.z = secondInCam(2);

    middleCV.x = middleCam(0);
    middleCV.y = middleCam(1);
    middleCV.z = middleCam(2);

    thirdCV.x = thirdInCam(0);
    thirdCV.y = thirdInCam(1);
    thirdCV.z = thirdInCam(2);

    tipCV.x = tipInCam(0);
    tipCV.y = tipInCam(1);
    tipCV.z = tipInCam(2);

    std::vector<cv::Point2d> points2D;

    cv::Point3d vector1; vector1 = secondCV - originCV;
    cv::Point3d vector2; vector2 = middleCV - secondCV;
    cv::Point3d vector3; vector3 = thirdCV - middleCV;
    cv::Point3d vector4; vector4 = tipCV - thirdCV;

    double mag1, mag2, mag3, mag4;

    mag1 = sqrt(pow(vector1.x,2)+pow(vector1.y,2)+pow(vector1.z,2));
    mag2 = sqrt(pow(vector2.x,2)+pow(vector2.y,2)+pow(vector2.z,2));
    mag3 = sqrt(pow(vector3.x,2)+pow(vector3.y,2)+pow(vector3.z,2));
    mag4 = sqrt(pow(vector4.x,2)+pow(vector4.y,2)+pow(vector4.z,2));

    vector1.x = vector1.x/mag1; vector1.y = vector1.y/mag1; vector1.z = vector1.z/mag1;
    vector2.x = vector2.x/mag2; vector2.y = vector2.y/mag2; vector2.z = vector2.z/mag2;
    vector3.x = vector3.x/mag3; vector3.y = vector3.y/mag3; vector3.z = vector3.z/mag3;
    vector4.x = vector4.x/mag4; vector4.y = vector4.y/mag4; vector4.z = vector4.z/mag4;

    needle3Dmodel.push_back(originCV);

    double step = 0.0001;
    int cont=1;
    int cont2=0;

    cv::Point3d currentVector = vector1;
    cv::Point3d currentStartingPoint = originCV;
    cv::Point3d currentDestinationPoint = secondCV;

    while(true)
    {
       cv::Point3d nextPoint;
       nextPoint.x = currentStartingPoint.x + (step * cont *  currentVector.x);
       nextPoint.y = currentStartingPoint.y + (step * cont *  currentVector.y);
       nextPoint.z = currentStartingPoint.z + (step * cont *  currentVector.z);
       cont++;

       needle3Dmodel.push_back(nextPoint);

       double currentDistance = sqrt(pow(nextPoint.x-currentDestinationPoint.x,2) + pow(nextPoint.y-currentDestinationPoint.y,2)+ pow(nextPoint.z-currentDestinationPoint.z,2));

       if(currentDistance<=step)
        {
           if(cont2 == 0)
            {
                cont = 1;
                currentVector = vector2;
                currentStartingPoint = secondCV;
                currentDestinationPoint = middleCV;
                cont2++;
                needle3Dmodel.push_back(secondCV);
            }
           else if(cont2 == 1)
           {
               cont = 1;
               currentVector = vector3;
               currentStartingPoint = middleCV;
               currentDestinationPoint = thirdCV;
               cont2++;
               needle3Dmodel.push_back(middleCV);
           }
           else if(cont2 == 2)
           {
               cont = 1;
               currentVector = vector4;
               currentStartingPoint = thirdCV;
               currentDestinationPoint = tipCV;
               cont2++;
               needle3Dmodel.push_back(thirdCV);
           }
           else
           {
            needle3Dmodel.push_back(tipCV);
            break;
           }
        }
       else
           needle3Dmodel.push_back(nextPoint);
    }

//    cv::Mat P1, D1;

//    FileStorage f_P1(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/P1.xml", FileStorage::READ); f_P1["P1"]>>P1;
//    FileStorage f_D1(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/D1.xml", FileStorage::READ); f_D1["D1"]>>D1;

    Mat P1_ = P1(cv::Rect(0,0,3,3));

    Mat t_vec = Mat(1,3, CV_64F, double(0));
    Mat r_vec = Mat(1,3, CV_64F, double(0));

    std::vector<cv::Point2d> framePoints2D;

    for(int i=0; i<needle3Dmodel.size();i++)
    {
        needle3Dmodel[i].x = needle3Dmodel[i].x * 1000;
        needle3Dmodel[i].y = needle3Dmodel[i].y * 1000;
        needle3Dmodel[i].z = needle3Dmodel[i].z * 1000;
    }

    cv::projectPoints(needle3Dmodel, t_vec, r_vec, P1_, D1, framePoints2D);

    stereoImages Images;
    GlobaleImages.frameLeft.copyTo(Images.frameLeft);

    Mat mx1, my1;

    FileStorage f_mx1(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/mx1.xml", FileStorage::READ); f_mx1["mx1"]>>mx1;
    FileStorage f_my1(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/my1.xml", FileStorage::READ); f_my1["my1"]>>my1;

    cv::Mat frameLeft_rect;

    frameLeft_rect.create( Images.frameLeft.size(), frameLeft_rect.type() );

    remap(Images.frameLeft, frameLeft_rect, mx1, my1, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );

    for(int i=0; i<framePoints2D.size();i++)
        circle(frameLeft_rect, framePoints2D[i], 2, Scalar(0,255,255), -1, 8, 0);

    circle(frameLeft_rect, framePoints2D[framePoints2D.size()-1], 2, Scalar(0,0,255), -1, 8, 0);

    reverse(needle3Dmodel.begin(),needle3Dmodel.end());

    //imshow("frame", frameLeft_rect);
    //video.write(GlobaleImages.frameLeft);
}

void VisualTrackingThread::showSlots()
{

    char text[255];
    sprintf(text, "%d", Curr_Slot);
    int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 0.5;
    int thickness = 1;

    // Compute slot positions
    Matrix4d cHm = Functions::ToolPose2Eigen(toolsTracker->Tools[0]);
    Matrix4d mHs = MAN_H_SLOTS_NEW_draw[Curr_Slot];
    Matrix4d cHs = cHm * mHs;

    std::vector<cv::Point3d> framePoints3D;
    std::vector<cv::Point2d> framePoints2D;
    cv::Point3d tmp;
    tmp.x = double(cHs(0,3)); tmp.y = double(cHs(1,3)); tmp.z = double(cHs(2,3));
    framePoints3D.push_back(tmp);

    Mat t_vec = Mat(1,3, CV_64F, double(0));
    Mat r_vec = Mat(1,3, CV_64F, double(0));

    Mat P1_ = P1(cv::Rect(0,0,3,3));
    Mat D1_ = Mat(4,1, CV_64F, double(0));
    cv::projectPoints(framePoints3D, t_vec, r_vec, P1_, D1_, framePoints2D);

//        cv::circle(GlobaleImages.frameLeft, framePoints2D[0], 5, Scalar(0,255,255), -1, 8, 0);
    cv::drawMarker(GlobaleImages.frameLeft, framePoints2D[0],  cv::Scalar(0,255,255), MARKER_CROSS, 5, 1);
    framePoints2D[0].y -=10;
    cv::putText(GlobaleImages.frameLeft, text, framePoints2D[0], fontFace, fontScale, Scalar::all(255), thickness,2);

}

void VisualTrackingThread::showHandEye(Mat3b frame, cv::Mat cHm)
{
    Vector4d origin(0,0,0,1);
    Vector4d x(0.02,0,0,1);
    Vector4d y(0,0.02,0,1);
    Vector4d z(0,0,0.02,1);

    Matrix4d  markerInHandeyeFrame = CV2EigenMat(cHm) ;
//    cout << "showHandEye markerInHandeyeFrame: " << markerInHandeyeFrame << endl;

    Vector4d originInCam = markerInHandeyeFrame*origin;
    Vector4d xCam = markerInHandeyeFrame*x;
    Vector4d yInCam = markerInHandeyeFrame*y;
    Vector4d zInCam = markerInHandeyeFrame*z;

    cv::Point3d originCV;
    cv::Point3d xCV;
    cv::Point3d yCV;
    cv::Point3d zCV;

    originCV.x = originInCam(0);
    originCV.y = originInCam(1);
    originCV.z = originInCam(2);
//    cout << "showHandEye originCV: " << originCV << endl;

    xCV.x = xCam(0);
    xCV.y = xCam(1);
    xCV.z = xCam(2);

    yCV.x = yInCam(0);
    yCV.y = yInCam(1);
    yCV.z = yInCam(2);

    zCV.x = zInCam(0);
    zCV.y = zInCam(1);
    zCV.z = zInCam(2);

    std::vector<cv::Point3d> framePoints3D;
    std::vector<cv::Point2d> framePoints2D;

    framePoints3D.push_back(originCV);
    framePoints3D.push_back(xCV);
    framePoints3D.push_back(yCV);
    framePoints3D.push_back(zCV);

    Mat t_vec = Mat(1,3, CV_64F, double(0));
    Mat r_vec = Mat(1,3, CV_64F, double(0));

    Mat P1_ = P1(cv::Rect(0,0,3,3));
    Mat D1_ = Mat(4,1, CV_64F, double(0));
    cv::projectPoints(framePoints3D, t_vec, r_vec, P1_, D1_, framePoints2D);

    line(GlobaleImages.frameLeft, framePoints2D[0], framePoints2D[1], Scalar(0,0,255), 2, 8, 0);
    line(GlobaleImages.frameLeft, framePoints2D[0], framePoints2D[2], Scalar(0,255,0), 2, 8, 0);
    line(GlobaleImages.frameLeft, framePoints2D[0], framePoints2D[3], Scalar(255,0,0), 2, 8, 0);

   }


void VisualTrackingThread::showNeedleDriverFrameInCam( cv::Mat &needleDriverPoseInCamCV, bool newHandEye )
{

    Vector4d origin(0,0,0,1);
    Vector4d x(0.025,0,0,1);
    Vector4d y(0,0.025,0,1);
    Vector4d z(0,0,0.025,1);

    Matrix4d  needleDriverPoseInCamEigen = CV2EigenMat( needleDriverPoseInCamCV ) ;

    Vector4d originInCam = needleDriverPoseInCamEigen*origin;
    Vector4d xCam = needleDriverPoseInCamEigen*x;
    Vector4d yInCam = needleDriverPoseInCamEigen*y;
    Vector4d zInCam = needleDriverPoseInCamEigen*z;

    cv::Point3d originCV;
    cv::Point3d xCV;
    cv::Point3d yCV;
    cv::Point3d zCV;

    originCV.x = originInCam(0);
    originCV.y = originInCam(1);
    originCV.z = originInCam(2);

    xCV.x = xCam(0);
    xCV.y = xCam(1);
    xCV.z = xCam(2);

    yCV.x = yInCam(0);
    yCV.y = yInCam(1);
    yCV.z = yInCam(2);

    zCV.x = zInCam(0);
    zCV.y = zInCam(1);
    zCV.z = zInCam(2);

    std::vector<cv::Point3d> framePoints3D;
    std::vector<cv::Point2d> framePoints2D;

    framePoints3D.push_back(originCV);
    framePoints3D.push_back(xCV);
    framePoints3D.push_back(yCV);
    framePoints3D.push_back(zCV);

//    cv::Mat M1, D1, mx1, my1, P1;

//    FileStorage f_M1(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/M1.xml", FileStorage::READ); f_M1["M1"]>>M1;
//    FileStorage f_D1(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/D1.xml", FileStorage::READ); f_D1["D1"]>>D1;
//    FileStorage f_mx1(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/mx1.xml", FileStorage::READ); f_mx1["mx1"]>>mx1;
//    FileStorage f_my1(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/my1.xml", FileStorage::READ); f_my1["my1"]>>my1;
//    FileStorage f_P1(SRC_FILES_DIR+"VisionSystem/caliInfo/steroCaliOutput/P1.xml", FileStorage::READ); f_P1["P1"]>>P1;


    Mat t_vec = Mat(1,3, CV_64F, double(0));
    Mat r_vec = Mat(1,3, CV_64F, double(0));


    Mat P1_ = P1(cv::Rect(0,0,3,3));
    cv::projectPoints(framePoints3D, t_vec, r_vec, P1_, D1, framePoints2D);

    stereoImages Images;
    GlobaleImages.frameLeft.copyTo(Images.frameLeft);
    cv::Mat frameLeft_rect;
    frameLeft_rect.create( Images.frameLeft.size(), frameLeft_rect.type() );
    remap(Images.frameLeft, frameLeft_rect, mx1, my1, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );

    //Drawing Frame
    if (!newHandEye)
    {
        line(Images.frameLeft, framePoints2D[0], framePoints2D[1], Scalar(0,0,255), 2, 8, 0);
        line(Images.frameLeft, framePoints2D[0], framePoints2D[2], Scalar(0,255,0), 2, 8, 0);
        line(Images.frameLeft, framePoints2D[0], framePoints2D[3], Scalar(255,0,0), 2, 8, 0);
        imshow("frame",Images.frameLeft);
    }
    else
    {
        line(frameLeft_rect, framePoints2D[0], framePoints2D[1], Scalar(0,0,255), 2, 8, 0);
        line(frameLeft_rect, framePoints2D[0], framePoints2D[2], Scalar(0,255,0), 2, 8, 0);
        line(frameLeft_rect, framePoints2D[0], framePoints2D[3], Scalar(255,0,0), 2, 8, 0);/*
        line(Images.frameLeft, framePoints2D[0], framePoints2D[1], Scalar(255,0,255), 2, 8, 0);
        line(Images.frameLeft, framePoints2D[0], framePoints2D[2], Scalar(0,255,255), 2, 8, 0);
        line(Images.frameLeft, framePoints2D[0], framePoints2D[3], Scalar(255,255,0), 2, 8, 0);*/
        //imshow("frame",frameLeft_rect);
    }

}

void VisualTrackingThread::checkImages()
{
        if (needleDetected == true)
        {
            stereoImages needleEnergies_show;
            Mat Eng_L_, Eng_R_;
            Eng_L_ = imread(ename_needleEnergyL, CV_LOAD_IMAGE_COLOR);
            Eng_R_ = imread(ename_needleEnergyR, CV_LOAD_IMAGE_COLOR);
            Eng_L_.copyTo(needleEnergies_show.frameLeft);
            Eng_R_.copyTo(needleEnergies_show.frameRight);
            emit showImagesInDialog(needleEnergies_show);
            needleDetected = false;
        }
}


void VisualTrackingThread::displayThread()
{
    // draw thread on images
    if (Inspect)
    {
        emit showThreadInDialog(GlobaleImages_show);
        Inspect = false;
    }
}

void VisualTrackingThread::showThread(int indx_slot)
{
    // draw thread on images
    if (Inspect)
    {
        Point3d pnt3_thread;
        Point3d pnt3_root;
        switch(indx_slot)
        {
            case 0: pnt3_root = Point3d(0.0583133,  -0.00505883,  0.157311); break;
            case 1: pnt3_root = Point3d(0.0598031,  0.0101367,  0.157311); break;
            case 2: pnt3_root = Point3d(0.0602805,  0.0127072,  0.157782); break;
            case 3: pnt3_root = Point3d(0.060517,  -0.0051924,  0.156845); break;
            default: pnt3_root = Point3d(0.0583133,  -0.00505883,  0.157311); break;
        }

        //Point3d pnt3_root(0.0583133,  -0.00505883,  0.157311); // 0.0586113,  -0.00505883,  0.157311; 0.0583133,  -0.00505883,  0.157311
        findThreadInImages(pnt3_root, pnt3_thread);
        emit showThreadInDialog(GlobaleImages_show);

        ofstream fstream_thread_new;
        fstream_thread_new.open(fname_thread_new);
        fstream_thread_new << pnt3_root.x << " " << pnt3_root.y << " " << pnt3_root.z<< endl;
        fstream_thread_new << pnt3_thread.x << " " << pnt3_thread.y << " " << pnt3_thread.z << endl;
        cout << "New thread pose is written to " << fname_thread_new << endl;
        fstream_thread_new.close();

        Inspect = false;
    }
}


void VisualTrackingThread::findThreadInImages(Point3d pnt3_root, Point3d &pnt3_tip)
{
    Mat leftImage_gray, rightImage_gray;
    cvtColor(GlobaleImages.frameLeft, leftImage_gray, CV_RGB2GRAY);
    cvtColor(GlobaleImages.frameRight, rightImage_gray, CV_RGB2GRAY);
    IplImage currFrame1cLeft, currFrame1cRight;
    currFrame1cLeft = leftImage_gray;
    currFrame1cRight = rightImage_gray;

    // Detect thread
    int x_l, x_r;
    needlePoseEstimation * threadEst = new needlePoseEstimation();
    threadEst->FindThread(&currFrame1cLeft, &currFrame1cRight, &x_l, &x_r);

    Point2d pnt2d_tip_l, pnt2d_tip_r;
    pnt2d_tip_l.x = x_l; pnt2d_tip_l.y = 40;
    pnt2d_tip_r.x = x_r; pnt2d_tip_r.y = 40;
    cout << "Thread left: " << pnt2d_tip_l
         << " right: " << pnt2d_tip_r  << endl;

    cv::circle(GlobaleImages_show.frameLeft, pnt2d_tip_l, 5, Scalar(255, 0, 255));
    cv::circle(GlobaleImages_show.frameRight, pnt2d_tip_r, 5, Scalar( 255,0, 255));

    // 2D -> 3D
    Mat point4D;
    triangulatePoints(P1, P2, Mat(pnt2d_tip_l), Mat(pnt2d_tip_r), point4D);

    pnt3_tip.x = point4D.at<double>(0,0)/point4D.at<double>(0,3);
    pnt3_tip.y = point4D.at<double>(0,1)/point4D.at<double>(0,3);
    pnt3_tip.z = point4D.at<double>(0,2)/point4D.at<double>(0,3);

    cout << "Thread root " << pnt3_root << endl;
    cout << "Thread tip " << pnt3_tip << endl;

    // 3D -> 2D
    vector<Point2d> pnt2_l, pnt2_r;
    vector<Point3d> pnt3;

    pnt3.push_back(pnt3_root);
    pnt3.push_back(pnt3_tip);

    Point3ToPoint2(pnt3, pnt2_l, pnt2_r);

    for (int i=0; i<pnt2_l.size(); i++)
    {
        cv::circle(GlobaleImages_show.frameLeft, pnt2_l[i], 5, Scalar(0, 255, 255));
        cv::circle(GlobaleImages_show.frameRight, pnt2_r[i], 5, Scalar(0, 255, 255));
        cout << "circle left " << pnt2_l[i] << " right " << pnt2_r[i] << endl;
    }
}

void VisualTrackingThread::Point3ToPoint2(vector<Point3d> &pnt3, vector<Point2d> &pnt2_l, vector<Point2d> &pnt2_r)
{
    Mat t_vec1 = Mat(3,1, CV_64F, double(0));
    Mat r_vec1 = Mat(3,1, CV_64F, double(0));
    Mat t_vec2 = Mat(3,1, CV_64F, double(0));
    Mat r_vec2 = Mat(3,1, CV_64F, double(0));    
    Mat D1_ = Mat(4,1, CV_64F, double(0));
    Mat D2_ = Mat(4,1, CV_64F, double(0));
    Mat P1_ = Mat(3,3, CV_64F, double(0));
    Mat P2_ = Mat(3,3, CV_64F, double(0));

    cv::Mat T_homogeneous(4,1,cv::DataType<double>::type); // translation vector
    cv::decomposeProjectionMatrix(P1, P1_, r_vec1, T_homogeneous);
    cv::decomposeProjectionMatrix(P2, P2_, r_vec2, T_homogeneous);

    t_vec2.at<double>(0,0) = T_homogeneous.at<double>(0,0)/T_homogeneous.at<double>(3,0);
    t_vec2.at<double>(1,0) = T_homogeneous.at<double>(1,0)/T_homogeneous.at<double>(3,0);
    t_vec2.at<double>(2,0) = T_homogeneous.at<double>(2,0)/T_homogeneous.at<double>(3,0);

    cv::projectPoints(pnt3, r_vec1, t_vec1, P1_, D1_, pnt2_l);
    cv::projectPoints(pnt3, r_vec2, -t_vec2, P2_, D2_, pnt2_r);

}

void VisualTrackingThread::saveAllNeedleResults(vector<vector<cv::Point3d> > needlePoints_all, needlePoseEstimation *pose3Dto2D)
{


    for (int i=0; i<needlePoints_all.size(); i++)
    {
        vector<cv::Point3d> needlePoints_tmp;
        for (int j=0; j<NeedlePoints.size(); j++)
        {
            needlePoints_tmp.push_back(needlePoints_all[i][j]);
        }


        Mat imageL, imageR;
        GlobaleImages.frameLeft.copyTo(imageL);
        GlobaleImages.frameRight.copyTo(imageR);

        pose3Dto2D->printNeedlePntsGlobalImage(imageL, imageR, needlePoints_tmp,
                                               toolIndx_detectneedle, 0, 255, 0);

        string fname_L = "./Data/needleAll_" + std::to_string(i) + "L.png";
        string fname_R = "./Data/needleAll_" + std::to_string(i) + "R.png";
        imwrite(fname_L.c_str(), imageL);
        imwrite(fname_R.c_str(), imageR);
    }

}
