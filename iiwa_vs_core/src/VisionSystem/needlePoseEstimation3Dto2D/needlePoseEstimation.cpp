#include "needlePoseEstimation.h"

using namespace std;

void needlePoseEstimation::calculateKernel( double sigma_in, int kernelDimension_in, int typeKernel_in,  CvMat * Kernel_out )
{
	for(int y=0; y < kernelDimension_in; y++)
		for(int x=0; x < kernelDimension_in; x++)
		{
			double elementValue;

			int newX = (x - (kernelDimension_in - 1)/2);
			int newY = (y - (kernelDimension_in - 1)/2);

			double exponent  = (pow((double) newX,2) + pow((double) newY,2)) / (2*pow(sigma_in, 2));

			//XX
			if(typeKernel_in == 1)
			{
				double base = (pow((double)newX, 2) - pow(sigma_in, 2)) / ( 2*pow(sigma_in, 6)*M_PI );
				elementValue = base * exp((-1)*exponent);
			}

			//XY
			if(typeKernel_in == 2)
			{		
				double base = (double)(newX * newY) / ( 2*pow(sigma_in, 6)*M_PI );
				elementValue = base * exp((-1)*exponent);
			}

			//YY
			if(typeKernel_in == 3)
			{			
				double base = (pow((double)newY, 2) - pow(sigma_in, 2)) / ( 2*pow(sigma_in, 6)*M_PI );
				elementValue = base * exp((-1)*exponent);
			}

			*( (double*)CV_MAT_ELEM_PTR( *Kernel_out, y, x ) ) = elementValue;
		}
}

void needlePoseEstimation::HessianAnalysisEigenvalues( IplImage * imageGray_in, double sigma_in, int kernelDimension_in, IplImage * imageGray_out )
{

	CvMat * imageMatrix = cvCreateMat(imageGray_in->height, imageGray_in->width, CV_64FC1);
	cvConvert( imageGray_in, imageMatrix );
	
	CvMat * kernelXX = cvCreateMat(kernelDimension_in, kernelDimension_in, CV_64FC1);
	calculateKernel(sigma_in, kernelDimension_in, 1, kernelXX);

	CvMat * kernelXY = cvCreateMat(kernelDimension_in, kernelDimension_in, CV_64FC1);
	calculateKernel(sigma_in, kernelDimension_in, 2, kernelXY);

	CvMat * kernelYY = cvCreateMat(kernelDimension_in, kernelDimension_in, CV_64FC1);
	calculateKernel(sigma_in, kernelDimension_in, 3, kernelYY);

	CvMat * Dxx = cvCreateMat(imageGray_in->height, imageGray_in->width, CV_64FC1);
	CvMat * Dxy = cvCreateMat(imageGray_in->height, imageGray_in->width, CV_64FC1);
	CvMat * Dyy = cvCreateMat(imageGray_in->height, imageGray_in->width, CV_64FC1);

	cvFilter2D(imageMatrix, Dxx, kernelXX);
	cvFilter2D(imageMatrix, Dxy, kernelXY);
	cvFilter2D(imageMatrix, Dyy, kernelYY);

	for( int y=0; y < imageGray_in->height; y++ )
	{
		double* vessel = (double*) ( imageGray_out->imageData + y * imageGray_out->widthStep);
			
		for( int x=0; x < imageGray_in->width; x++ )
		{
			double maxEigenvalue = 0;
			double minEigenvalue = 0;

			double elementDxx = CV_MAT_ELEM( *Dxx, double, y, x );
			double elementDxy = CV_MAT_ELEM( *Dxy, double, y, x );
			double elementDyy = CV_MAT_ELEM( *Dyy, double, y, x );

			//Find Eigenvalues
			double eig1 =  (double)(0.5*( elementDxx + elementDyy + sqrt( (double)( pow(elementDxx - elementDyy, 2) + 4.0*pow(elementDxy,2) )) ));
			double eig2 =  (double)(0.5*( elementDxx + elementDyy - sqrt( (double)( pow(elementDxx - elementDyy, 2) + 4.0*pow(elementDxy,2) )) ));

			//Analyze Eigenvalues' value
			if( abs(eig1) > abs(eig2) )
			{
				minEigenvalue = eig2;
				maxEigenvalue = eig1;
			}
			else
			{
				minEigenvalue = eig1;
				maxEigenvalue = eig2;
			}

			//Vesselness - Hessian Matrix and Eigen analysis
			if( maxEigenvalue < 0 )
			{
				vessel[x] = 0.0;
			}
			else
			{
				//Find Eigenvector of the smallest eigenvalue - Guidewire direction
				double tmp_eq1 = 1.0;
				double tmp_eq2 = (eig1 - elementDxx)/elementDxy;		
				double module_orientation = sqrt( pow(tmp_eq1,2) + pow(tmp_eq2,2) );			
				double ux1_tmp = tmp_eq1 / module_orientation;
				double uy1_tmp = tmp_eq2 / module_orientation;
				double ux1 = (-1.0)*uy1_tmp;
				double uy1 = ux1_tmp;

				if( abs(eig1) < abs(eig2) )
				{
					ux1 = ux1_tmp;	uy1 = uy1_tmp;
				}

				//Vesselness
				vessel[x] = maxEigenvalue;
			}
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

void needlePoseEstimation::calcFeatureImages( IplImage * leftImage_in, IplImage * rightImage_in, IplImage * leftImageDistance_in, IplImage * rightImageDistance_in )
{
	HessianAnalysisEigenvalues( leftImage_in, 2.0, 13, leftImageFeature );
	HessianAnalysisEigenvalues( rightImage_in, 2.0, 13, rightImageFeature );

//    cvSaveImage("EngergyL.png",energyIPL_L);
//    cvSaveImage("EngergyR.png",energyIPL_R);

//    cvNamedWindow( "Engergy L" );
//    cvShowImage( "Engergy L", image_L );

//    cvNamedWindow( "Engergy R" );
//    cvShowImage( "Engergy R", image_R );

	//cv::distanceTransform( leftImage_in, leftImageDistance_in, leftImageDistance_in
}



void needlePoseEstimation::calcEnergy( std::vector<cv::Point3d> needle3DModelCurrentPose_in, double * energy_out, int toolIndx )
{
    std::vector<cv::Point3d> points3DModelCamera;
    std::vector<cv::Point2d> projected2DLeft;
    std::vector<cv::Point2d> projected2DlRight;
// test comment
	double featureImage = 0.0;
    projectNeedlePntsFrameLR(needle3DModelCurrentPose_in, points3DModelCamera, projected2DLeft, projected2DlRight, toolIndx );

    vector<double> tmp;
    tmp.resize(projected2DlRight.size());
    for(int i=1; i < projected2DlRight.size(); i++)
    {
        tmp[i] = ((double *)(rightImageFeature->imageData +
                               (int)(projected2DlRight[i].y)*rightImageFeature->widthStep))[(int)(projected2DlRight[i].x)];
//        if (i==0)
//        {
//            if (tmp == 0)
//                tmp = 10;
//            else
//            {
//                tmp = 0;
//            }
//        }
//        else
        {
            if (tmp[i] == 0)
            {
                tmp[i] = -10;
//                if (i>3 && tmp[i-1] <= 0)
//                    tmp[i] = -20; //penalty for continuous 0
            }
            else
            {
                tmp[i] = 10;
            }
        }

        featureImage += tmp[i];
    }

    for(int i=1; i < projected2DLeft.size(); i++)
    {
        tmp[i] =  ((double *)(leftImageFeature->imageData +
                 (int)(projected2DLeft[i].y)*leftImageFeature->widthStep))[(int)(projected2DLeft[i].x)];

//        if (i==0)
//        {
//            if (tmp == 0)
//                tmp = 10;
//            else
//            {
//                tmp = 0;
//            }
//        }
//        else
        {
            if (tmp[i] == 0)
            {
                tmp[i] = -10;
//                if (i>3 && tmp[i-1] <= 0)
//                    tmp[i] = -20; //penalty for continuous 0
            }
            else
            {
                tmp[i] = 10;
            }
        }

        featureImage += tmp[i];
    }

	//add regularization term in the overall energy

    *energy_out = featureImage;
}

void needlePoseEstimation::calcEnergy2(std::vector<cv::Point3d> needle3DModelCurrentPose_in, int toolIndx)
{
    std::vector<cv::Point3d> points3DModelCamera;
    std::vector<cv::Point2d> projected2DLeft;
    std::vector<cv::Point2d> projected2DlRight;

    double featureImage = 0.0;
    projectNeedlePntsFrameLR(needle3DModelCurrentPose_in, points3DModelCamera, projected2DLeft, projected2DlRight, toolIndx );

    for(int i=0; i < projected2DlRight.size(); i++)
    {
        featureImage = ((double *)(rightImageFeature->imageData +
                        (int)(projected2DlRight[i].y)*rightImageFeature->widthStep))[(int)(projected2DlRight[i].x)];
        energy_R.push_back(featureImage);
    }

    for(int i=0; i < projected2DLeft.size(); i++)
    {
        featureImage = ((double *)(leftImageFeature->imageData +
                        (int)(projected2DLeft[i].y)*leftImageFeature->widthStep))[(int)(projected2DLeft[i].x)];
        energy_L.push_back((featureImage));
    }
}



void needlePoseEstimation::optimisation( std::vector<cv::Point3d> &needle3DModel_inout, cv::Mat * needlePose_inout, int toolIndx)
{
    //For debug--------
    string ename_l = ("Data/EngergyL_hist" );
    string ename_r = ("Data/EngergyR_hist");

    ofstream f_energy_l, f_energy_r;
    f_energy_l.open(ename_l);
    f_energy_r.open(ename_r);
    // -----------------

    double maxTranslationX = (15.0/1000.0);
    double maxRotationX = (M_PI/180*60); // 180*5
    double maxRotationY = (M_PI/180*60);
    double maxRotationZ = (M_PI/180*40);
    double maxEnergy = DBL_MIN;

    int numberStepVariablesTranslation = 10;//100;
    int numberStepVariablesRotationX = 30;//100; 30
    int numberStepVariablesRotationY = 30;//100;
    int numberStepVariablesRotationZ = 20;//100;


    cv::Mat nHn_ = cv::Mat(4, 4, CV_64F, double(0));
    cv::Mat cHnn_ = cv::Mat(4, 4, CV_64F, double(0));
    cv::Mat cHn = cv::Mat::eye(4, 4, CV_64F); // 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -0.012, 0, 0, 0, 1);
    cHn.at<double>(2, 3) = -0.012;
//    cHn.at<double>(2, 3) = -0.02;
    cv::Mat nHp = cv::Mat(4, 1, CV_64F, double(0));
    cv::Mat tHp = cv::Mat(4, 1, CV_64F, double(0)); // points to tool
    cv::Mat tHn;


    // For tool R
    double tmp[16] = {    -0.9954,   -0.0888,   -0.0360,    0.0001,
                          -0.0598,    0.2814,    0.9576,   -0.0071,
                          -0.0750,    0.9554,   -0.2855,    0.0134,
                                0,         0,         0,    1.0000};

    cv::Mat iHn = cv::Mat(4, 4, CV_64F, tmp);
    cv::Mat iHnn_ = cv::Mat::eye(4, 4, CV_64F);

    if (toolIndx == 1)
    {
        lHn.copyTo(tHn);        
    }
    else
    {
        rHn.copyTo(tHn);
    }

//    cout << "tHn " << tHn << endl;

	std::vector<cv::Mat> newPoses;
    std::vector<std::vector<cv::Point3d> > newPosesNeedle3DModel;


for (int d=0; d < numberStepVariablesRotationX; d++)
    for(int a=0; a < numberStepVariablesTranslation; a++)
            for (int c=0; c< numberStepVariablesRotationZ; c++)
                        for(int b=0; b < numberStepVariablesRotationY; b++)
//    for(int a=0; a < numberStepVariablesTranslation/2; a++)
//        for(int b=0; b < numberStepVariablesRotationY/2; b++)
//            for (int c=0; c< numberStepVariablesRotationZ/2; c++)
//            for (int d=0; d < numberStepVariablesRotationX/2; d++)
            {
                double translationX = -(maxTranslationX*((a*1.0)/(numberStepVariablesTranslation*1.0)))+ maxTranslationX/2;//*(-1.0);
                double angleRotationX = maxRotationX*((numberStepVariablesRotationX-d)/(numberStepVariablesRotationX*1.0))- (maxRotationX/2.0);
                double angleRotationY = maxRotationY*((b*1.0)/(numberStepVariablesRotationY*1.0))- (maxRotationY/2.0);
                double angleRotationZ = maxRotationZ*((c*1.0)/(numberStepVariablesRotationZ*1.0))- (maxRotationZ/2.0);


//                double translationX = -(maxTranslationX/2*((a*1.0)/(numberStepVariablesTranslation*0.5)));
//                double angleRotationX = -maxRotationX/2*((d*1.0)/(numberStepVariablesRotationX*0.5));
//                double angleRotationY = -maxRotationY/2*((b*1.0)/(numberStepVariablesRotationY*0.5));
//                double angleRotationZ = -maxRotationZ/2*((c*1.0)/(numberStepVariablesRotationZ*0.5));

                std::vector<cv::Point3d> newPoseNeedle3DModel;

                double cx, cy, cz, sx, sy, sz, tx;
                cx = cos(angleRotationX );
                cy = cos(angleRotationY );
                cz = cos(angleRotationZ );

                sx = sin(angleRotationX );
                sy = sin(angleRotationY );
                sz = sin(angleRotationZ );

                tx = translationX ;

                if (toolIndx == 1)
                {
                    nHn_.at<double>(0, 0) = cy * cz;   nHn_.at<double>(0, 1) = -cy * sz; nHn_.at<double>(0, 2) = sy;  nHn_.at<double>(0, 3) = tx;
                    nHn_.at<double>(1, 0) = sz;        nHn_.at<double>(1, 1) = cz;       nHn_.at<double>(1, 2) = 0.0; nHn_.at<double>(1, 3) = 0;
                    nHn_.at<double>(2, 0) = -cz* sy;   nHn_.at<double>(2, 1) = sy * sz;  nHn_.at<double>(2, 2) = cy;  nHn_.at<double>(2, 3) = 0.0;
                    nHn_.at<double>(3, 0) = 0.0;       nHn_.at<double>(3, 1) = 0.0;      nHn_.at<double>(3, 2) = 0.0; nHn_.at<double>(3, 3) = 1.0;
                }
                else
                {
                    iHnn_.at<double>(0, 0) = cy * cz;   iHnn_.at<double>(0, 1) = -cy * sz; iHnn_.at<double>(0, 2) = sy;  iHnn_.at<double>(0, 3) = tx;
                    iHnn_.at<double>(1, 0) = sz;        iHnn_.at<double>(1, 1) = cz;       iHnn_.at<double>(1, 2) = 0.0; iHnn_.at<double>(1, 3) = 0;
                    iHnn_.at<double>(2, 0) = -cz* sy;   iHnn_.at<double>(2, 1) = sy * sz;  iHnn_.at<double>(2, 2) = cy;  iHnn_.at<double>(2, 3) = 0.0;
                    iHnn_.at<double>(3, 0) = 0.0;       iHnn_.at<double>(3, 1) = 0.0;      iHnn_.at<double>(3, 2) = 0.0; iHnn_.at<double>(3, 3) = 1.0;
                    nHn_ = iHn.inv() * iHnn_ * iHn;
                }

                cHnn_.at<double>(0, 0) = 1.0;   cHnn_.at<double>(0, 1) = 0.0;   cHnn_.at<double>(0, 2) = 0.0; cHnn_.at<double>(0, 3) = 0;
                cHnn_.at<double>(1, 0) = 0.0;   cHnn_.at<double>(1, 1) = cx;    cHnn_.at<double>(1, 2) = -sx; cHnn_.at<double>(1, 3) = 0;
                cHnn_.at<double>(2, 0) = 0.0;   cHnn_.at<double>(2, 1) = sx;    cHnn_.at<double>(2, 2) = cx;  cHnn_.at<double>(2, 3) = 0;
                cHnn_.at<double>(3, 0) = 0.0;   cHnn_.at<double>(3, 1) = 0.0;   cHnn_.at<double>(3, 2) = 0.0; cHnn_.at<double>(3, 3) = 1.0;

                nHn_ = nHn_ * cHn.inv() * cHnn_ * cHn;


                for(int d=0; d < needle3DModel_inout.size(); d++)
                {
                    cv::Point3d currentPoint_Point3d;

                    nHp.at<double>(0,0) = needle3DModel_inout[d].x;
                    nHp.at<double>(1,0) = needle3DModel_inout[d].y;
                    nHp.at<double>(2,0) = needle3DModel_inout[d].z;
                    nHp.at<double>(3,0) = 1;

                    tHp =  tHn * nHn_ * nHp;

                    currentPoint_Point3d.x = tHp.at<double>(0,0);// /tHp.at<double>(3,0);
                    currentPoint_Point3d.y = tHp.at<double>(1,0);// /tHp.at<double>(3,0);
                    currentPoint_Point3d.z = tHp.at<double>(2,0);// /tHp.at<double>(3,0);

                    newPoseNeedle3DModel.push_back(currentPoint_Point3d);
                }

                newPosesNeedle3DModel.push_back(newPoseNeedle3DModel);
                newPoses.push_back(tHn * nHn_);

//                cout << 180*(angleRotationX - (maxRotationX/2.0))/3.14 << " ";
            }



    int bestIndex = 0;
    int nbNeedlePoints = needle3DModel_inout.size();
	needle3DModel_inout.clear();
    maxEnergy = (nbNeedlePoints-1)*(-10)*2;
	
//    for(int i=0; i < newPoses.size(); i++)
//    {
////        int i =  190000;
//        double energy;
//        calcEnergy( newPosesNeedle3DModel[i], &energy, toolIndx );

//        f_energy_l  << energy << endl;

//        if( energy > maxEnergy )
//        {
//            maxEnergy = energy;
//            bestIndex = i;
//        }
//        if (energy >=100)
//            cout << "i " << i << " " << energy << endl;
//        if (energy == (nbNeedlePoints-1)*10*2)
//            break;
//    }
//    f_energy_l.close();

    vector<double> energy_vec;
    for(int i=0; i < newPoses.size(); i++)
    {
//        int i =  190000;
        double energy;
        calcEnergy( newPosesNeedle3DModel[i], &energy, toolIndx );

        energy_vec.push_back(energy);
        f_energy_l  << energy << endl;

        if( energy > maxEnergy )
        {
            maxEnergy = energy;
            bestIndex = i;
        }
        if (energy >=100)
            cout << "i " << i << " " << energy << endl;
        if (energy == (nbNeedlePoints-1)*10*2)
            break;
    }
    f_energy_l.close();

    for (int i=0; i<energy_vec.size(); i++)
    {
        if (energy_vec[i] == maxEnergy)
        {
            for (int j=0; j<newPosesNeedle3DModel[0].size(); j++)
            {
                needle3DModel_inout.push_back(newPosesNeedle3DModel[i][j]);
            }
            needle3D_all.push_back(needle3DModel_inout);
            needle3DModel_inout.clear();
        }

     }


    // -------------------------------------------------------------------
    needle3DModel_inout.clear();
	for(int i=0; i < newPosesNeedle3DModel[bestIndex].size(); i++)
		needle3DModel_inout.push_back(newPosesNeedle3DModel[bestIndex][i]);

    *needlePose_inout = newPoses[bestIndex];

    cout << "tHn new " << newPoses[bestIndex] << endl;
    std::cout << "TopIndex=" << bestIndex << " maxEnergy " << maxEnergy << std::endl;


    // For debug: Save energy image ----------------------------------
    energyIPL_L = cvCloneImage(leftImageFeature);
    energyIPL_R = cvCloneImage(rightImageFeature);

    cvConvertScale(energyIPL_L, energyIPL_L, 2550, 0); //255
    cvConvertScale(energyIPL_R, energyIPL_R, 2550, 0);

    char text[255];
    sprintf(text, "%d %f", bestIndex, maxEnergy);

    CvFont font;
    double fontScale = 1;
    int thickness = 3;
    cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, fontScale,fontScale,0,thickness);

    cv::Point textOrg(0, 440);
    cvPutText (energyIPL_L,text,textOrg, &font, cvScalar(255,0,0));

    cv::cvarrToMat(energyIPL_L).copyTo(energyMat_L);
    cv::cvarrToMat(energyIPL_R).copyTo(energyMat_R);

    std::vector<cv::Point3d> needle2camera;
    std::vector<cv::Point2d> needle2imgL, needle2imgR;

    projectNeedlePntsFrameLR(needle3DModel_inout, needle2camera, needle2imgL, needle2imgR, toolIndx);
    for(int i=0; i<needle2imgL.size(); i++)
    {
            cv::circle(energyMat_L, needle2imgL[i], 5, cvScalar(255,0,255));
            cv::circle(energyMat_R, needle2imgR[i], 5, cvScalar(255,0,255));
    }
// =================================================================================================

    energy_L.clear();
    energy_R.clear();
    calcEnergy2(newPosesNeedle3DModel[bestIndex], toolIndx );
}

void needlePoseEstimation::projectNeedlePntsFrameLR(std::vector<cv::Point3d> needlepointsTool,
                                                    std::vector<cv::Point3d> &needle2camera,
                                                    std::vector<cv::Point2d> &needle2imgL,
                                                    std::vector<cv::Point2d> &needle2imgR,
                                                    int toolIndx)
{
    cv::Point3d pnt2Camera;
    cv::Mat tHp, cHp, cHt;

    //tHp = cv::Mat::eye(4,4,CV_64F);
    tHp = cv::Mat(4, 1, CV_64F, double(0));
    cHp = cv::Mat(4, 1, CV_64F, double(0));

    if (toolIndx == 1)
        cHl.copyTo(cHt);
    if (toolIndx == 2)
        cHr.copyTo(cHt);

    needle2camera.clear();
    needle2imgL.clear();
    needle2imgR.clear();

    for (int i=0; i<needlepointsTool.size(); i++)
    {
        tHp.at<double>(0,0) = needlepointsTool[i].x;
        tHp.at<double>(1,0) = needlepointsTool[i].y;
        tHp.at<double>(2,0) = needlepointsTool[i].z;
        tHp.at<double>(3,0) = 1;

        cHp = cHt * tHp;

        pnt2Camera.x = cHp.at<double>(0,0);
        pnt2Camera.y = cHp.at<double>(1,0);
        pnt2Camera.z = cHp.at<double>(2,0);

        needle2camera.push_back(pnt2Camera);
    }

    cv::Mat t_vec1 = cv::Mat(3,1, CV_64F, double(0));
    cv::Mat r_vec1 = cv::Mat(3,1, CV_64F, double(0));
    cv::Mat t_vec2 = cv::Mat(3,1, CV_64F, double(0));
    cv::Mat r_vec2 = cv::Mat(3,1, CV_64F, double(0));

    cv::Mat P1_ = cv::Mat(3,3, CV_64F, double(0));
    cv::Mat P2_ = cv::Mat(3,3, CV_64F, double(0));
    cv::Mat D1_ = cv::Mat(4,1, CV_64F, double(0));
    cv::Mat D2_ = cv::Mat(4,1, CV_64F, double(0));

    cv::Mat T_homogeneous(4,1,cv::DataType<double>::type);

    cv::decomposeProjectionMatrix(P1, P1_, r_vec1, T_homogeneous);
    cv::decomposeProjectionMatrix(P2, P2_, r_vec2, T_homogeneous);

    t_vec2.at<double>(0,0) = T_homogeneous.at<double>(0,0)/T_homogeneous.at<double>(3,0);
    t_vec2.at<double>(1,0) = T_homogeneous.at<double>(1,0)/T_homogeneous.at<double>(3,0);
    t_vec2.at<double>(2,0) = T_homogeneous.at<double>(2,0)/T_homogeneous.at<double>(3,0);

    cv::projectPoints(needle2camera, r_vec1, t_vec1, P1_, D1_, needle2imgL);
    cv::projectPoints(needle2camera, r_vec2, -t_vec2, P2_, D2_, needle2imgR);

}

void needlePoseEstimation::FindThread(IplImage * leftImage_in, IplImage * rightImage_in, int *X_l, int *X_r)
{
    leftImage = leftImage_in;
    rightImage = rightImage_in;

    leftImageFeature = cvCreateImage( cvGetSize(leftImage_in), IPL_DEPTH_64F, 1);
    rightImageFeature = cvCreateImage( cvGetSize(rightImage_in), IPL_DEPTH_64F, 1);
    leftImageFeatureDistance = cvCreateImage( cvGetSize(leftImage_in), IPL_DEPTH_64F, 1);
    rightImageFeatureDistance = cvCreateImage( cvGetSize(rightImage_in), IPL_DEPTH_64F, 1);

    calcFeatureImages( leftImage_in, rightImage_in, leftImageFeatureDistance, rightImageFeatureDistance);

    double tmp = 0;

    for(int x=1; x < leftImageFeature->width; x++)
    {
        tmp = ((double *)(leftImageFeature->imageData +
                               40 * leftImageFeature->widthStep))[x];
        //cout << x << " " << tmp << endl;
        if (tmp > 1 && x >100)
        {
            *X_l = x;
            break;
        }
    }

    for(int x=1; x < rightImageFeature->width; x++)
    {
        tmp = ((double *)(rightImageFeature->imageData +
                               40 * rightImageFeature->widthStep))[x];
        if (tmp > 1 && x >100)
        {
            *X_r = x;
            break;
        }
    }

    //cout << "Thread left: " << *X_l << " right: " << *X_r << endl;

    cvReleaseImage(&leftImageFeature);
    cvReleaseImage(&rightImageFeature);
    cvReleaseImage(&leftImageFeatureDistance);
    cvReleaseImage(&rightImageFeatureDistance);
}

void needlePoseEstimation::estimatePose( IplImage * leftImage_in, IplImage * rightImage_in,
                                         std::vector<cv::Point3d> &needle3DModel_inout,
                                         cv::Mat * needlePose_inout, int toolIndx)
{
    leftImage = leftImage_in;
    rightImage = rightImage_in;

	leftImageFeature = cvCreateImage( cvGetSize(leftImage_in), IPL_DEPTH_64F, 1);	
	rightImageFeature = cvCreateImage( cvGetSize(rightImage_in), IPL_DEPTH_64F, 1);	
	leftImageFeatureDistance = cvCreateImage( cvGetSize(leftImage_in), IPL_DEPTH_64F, 1);
	rightImageFeatureDistance = cvCreateImage( cvGetSize(rightImage_in), IPL_DEPTH_64F, 1);

    calcFeatureImages( leftImage_in, rightImage_in, leftImageFeatureDistance, rightImageFeatureDistance);

    optimisation( needle3DModel_inout, needlePose_inout, toolIndx );

    //printNeedlePntsFrameLR(needle3DModel_inout);

    cvReleaseImage(&leftImageFeature);
    cvReleaseImage(&rightImageFeature);
    cvReleaseImage(&leftImageFeatureDistance);
    cvReleaseImage(&rightImageFeatureDistance);
}


void needlePoseEstimation::printNeedlePntsGlobalImage(cv::Mat frameL, cv::Mat frameR, std::vector<cv::Point3d> needlepointsTool,
                                                      int toolIndx, int r, int g, int b)
{
    std::vector<cv::Point3d> needle2camera;
    std::vector<cv::Point2d> needle2imgL;
    std::vector<cv::Point2d> needle2imgR;

    cv::Point3d pnt2Camera;
    cv::Mat tHp, cHp, cHt;

    if (toolIndx == 1)
        cHl.copyTo(cHt);
    else
        cHr.copyTo(cHt);
cout << "cHt " << endl << cHt << endl;

    tHp = cv::Mat::eye(4,4,CV_64F);

    needle2camera.clear();
    needle2imgL.clear();
    needle2imgR.clear();

    for (int i=0; i<needlepointsTool.size(); i++)
    {
        tHp.at<double>(0,3) = needlepointsTool[i].x;
        tHp.at<double>(1,3) = needlepointsTool[i].y;
        tHp.at<double>(2,3) = needlepointsTool[i].z;

        cHp = cHt * tHp;

        pnt2Camera.x = cHp.at<double>(0,3);
        pnt2Camera.y = cHp.at<double>(1,3);
        pnt2Camera.z = cHp.at<double>(2,3);

        needle2camera.push_back(pnt2Camera);
    }

//    std::cout << "needle2camera " << needle2camera[0].x
//         << " " << needle2camera[0].y
//         << " " << needle2camera[0].z << std::endl;

    cv::Mat t_vec1 = cv::Mat(3,1, CV_64F, double(0));
    cv::Mat r_vec1 = cv::Mat(3,1, CV_64F, double(0));
    cv::Mat t_vec2 = cv::Mat(3,1, CV_64F, double(0));
    cv::Mat r_vec2 = cv::Mat(3,1, CV_64F, double(0));

    cv::Mat P1_ = cv::Mat(3,3, CV_64F, double(0));
    cv::Mat P2_ = cv::Mat(3,3, CV_64F, double(0));    
    cv::Mat D1_ = cv::Mat(4,1, CV_64F, double(0));
    cv::Mat D2_ = cv::Mat(4,1, CV_64F, double(0));

    cv::Mat T_homogeneous(4,1,cv::DataType<double>::type);

    cv::decomposeProjectionMatrix(P1, P1_, r_vec1, T_homogeneous);
    cv::decomposeProjectionMatrix(P2, P2_, r_vec2, T_homogeneous);

    t_vec2.at<double>(0,0) = T_homogeneous.at<double>(0,0)/T_homogeneous.at<double>(3,0);
    t_vec2.at<double>(1,0) = T_homogeneous.at<double>(1,0)/T_homogeneous.at<double>(3,0);
    t_vec2.at<double>(2,0) = T_homogeneous.at<double>(2,0)/T_homogeneous.at<double>(3,0);

    cv::projectPoints(needle2camera, r_vec1, t_vec1, P1_, D1_, needle2imgL);
    cv::projectPoints(needle2camera, r_vec2, -t_vec2, P2_, D2_, needle2imgR);

    for(int i=0; i<needle2imgL.size(); i++)
    {
            cv::circle(frameL, needle2imgL[i], 1, cvScalar(r,g,b));//, -1, 8, 0);
            cv::circle(frameR, needle2imgR[i], 1, cvScalar(r,g,b));//, -1, 8, 0);
    }
    for(int i=0; i<needle2imgL.size()-1; i++)
    {
        cv::line(frameL, needle2imgL[i], needle2imgL[i+1], cvScalar(r,g,b), 1);
        cv::line(frameR, needle2imgR[i], needle2imgR[i+1], cvScalar(r,g,b), 1);
    }
    cv::circle(frameL, needle2imgL[0], 2, cvScalar(0,0,255), -1, 8, 0);
    cv::circle(frameR, needle2imgR[0], 2, cvScalar(0,0,255), -1, 8, 0);
}

void needlePoseEstimation::printNeedleSearchRange(cv::Mat frameL, cv::Mat frameR, std::vector<cv::Point3d> needlepointsOpt,
                                                  int toolIndx, int r, int g, int b)
{
    double maxTranslationX = (0.0/1000.0); //15
    double maxRotationX = (M_PI/180*15); // 180*5
    double maxRotationY = (M_PI/180*60);
    double maxRotationZ = (M_PI/180*40);
    double maxEnergy = DBL_MIN;

    int numberStepVariablesTranslation = 1;//100;
    int numberStepVariablesRotationX = 1;//100; 5
    int numberStepVariablesRotationY = 1;//100;
    int numberStepVariablesRotationZ = 1;//100;


    cv::Mat nHn_ = cv::Mat(4, 4, CV_64F, double(0));
    cv::Mat cHnn_ = cv::Mat(4, 4, CV_64F, double(0));
    cv::Mat cHn = cv::Mat::eye(4, 4, CV_64F);
    cHn.at<double>(2, 3) = -0.012;
//    cHn.at<double>(2, 3) = -0.02;
    cv::Mat nHp = cv::Mat(4, 1, CV_64F, double(0));
    cv::Mat tHp = cv::Mat(4, 1, CV_64F, double(0)); // points to tool
    cv::Mat tHn;

    // For tool R
    double tmp[16] = {    -0.9954,   -0.0888,   -0.0360,    0.0001,
                          -0.0598,    0.2814,    0.9576,   -0.0071,
                          -0.0750,    0.9554,   -0.2855,    0.0134,
                                0,         0,         0,    1.0000};

    cv::Mat iHn = cv::Mat(4, 4, CV_64F, tmp);
    cv::Mat iHnn_ = cv::Mat::eye(4, 4, CV_64F);

    if (toolIndx == 1)
    {
        lHn.copyTo(tHn);
    }
    else
    {
        rHn.copyTo(tHn);
    }

    std::vector<cv::Mat> newPoses;
    std::vector<std::vector<cv::Point3d> > newPosesNeedle3DModel;

    for(int a=0; a <= numberStepVariablesTranslation; a++)
            for (int d=0; d <= numberStepVariablesRotationX; d++)
            for (int c=0; c<= numberStepVariablesRotationZ; c++)
                        for(int b=0; b <= numberStepVariablesRotationY; b++)
            {
                double translationX = -(maxTranslationX*((a*1.0)/(numberStepVariablesTranslation*1.0)))+ maxTranslationX/2;//*(-1.0);
                double angleRotationX = maxRotationX*((numberStepVariablesRotationX-d)/(numberStepVariablesRotationX*1.0))- (maxRotationX/2.0);
                double angleRotationY = maxRotationY*((b*1.0)/(numberStepVariablesRotationY*1.0))- (maxRotationY/2.0);
                double angleRotationZ = maxRotationZ*((c*1.0)/(numberStepVariablesRotationZ*1.0))- (maxRotationZ/2.0);

                std::vector<cv::Point3d> newPoseNeedle3DModel;
                double cx, cy, cz, sx, sy, sz, tx;
                cx = cos(angleRotationX );
                cy = cos(angleRotationY );
                cz = cos(angleRotationZ );

                sx = sin(angleRotationX );
                sy = sin(angleRotationY );
                sz = sin(angleRotationZ );

                tx = translationX ;

                if (toolIndx == 1)
                {
                    nHn_.at<double>(0, 0) = cy * cz;   nHn_.at<double>(0, 1) = -cy * sz; nHn_.at<double>(0, 2) = sy;  nHn_.at<double>(0, 3) = tx;
                    nHn_.at<double>(1, 0) = sz;        nHn_.at<double>(1, 1) = cz;       nHn_.at<double>(1, 2) = 0.0; nHn_.at<double>(1, 3) = 0;
                    nHn_.at<double>(2, 0) = -cz* sy;   nHn_.at<double>(2, 1) = sy * sz;  nHn_.at<double>(2, 2) = cy;  nHn_.at<double>(2, 3) = 0.0;
                    nHn_.at<double>(3, 0) = 0.0;       nHn_.at<double>(3, 1) = 0.0;      nHn_.at<double>(3, 2) = 0.0; nHn_.at<double>(3, 3) = 1.0;
                }
                else
                {
                    iHnn_.at<double>(0, 0) = cy * cz;   iHnn_.at<double>(0, 1) = -cy * sz; iHnn_.at<double>(0, 2) = sy;  iHnn_.at<double>(0, 3) = tx;
                    iHnn_.at<double>(1, 0) = sz;        iHnn_.at<double>(1, 1) = cz;       iHnn_.at<double>(1, 2) = 0.0; iHnn_.at<double>(1, 3) = 0;
                    iHnn_.at<double>(2, 0) = -cz* sy;   iHnn_.at<double>(2, 1) = sy * sz;  iHnn_.at<double>(2, 2) = cy;  iHnn_.at<double>(2, 3) = 0.0;
                    iHnn_.at<double>(3, 0) = 0.0;       iHnn_.at<double>(3, 1) = 0.0;      iHnn_.at<double>(3, 2) = 0.0; iHnn_.at<double>(3, 3) = 1.0;
                    nHn_ = iHn.inv() * iHnn_ * iHn;
                }

                cHnn_.at<double>(0, 0) = 1.0;   cHnn_.at<double>(0, 1) = 0.0;   cHnn_.at<double>(0, 2) = 0.0; cHnn_.at<double>(0, 3) = 0;
                cHnn_.at<double>(1, 0) = 0.0;   cHnn_.at<double>(1, 1) = cx;    cHnn_.at<double>(1, 2) = -sx; cHnn_.at<double>(1, 3) = 0;
                cHnn_.at<double>(2, 0) = 0.0;   cHnn_.at<double>(2, 1) = sx;    cHnn_.at<double>(2, 2) = cx;  cHnn_.at<double>(2, 3) = 0;
                cHnn_.at<double>(3, 0) = 0.0;   cHnn_.at<double>(3, 1) = 0.0;   cHnn_.at<double>(3, 2) = 0.0; cHnn_.at<double>(3, 3) = 1.0;

                nHn_ = nHn_ * cHn.inv() * cHnn_ * cHn;


                for(int d=0; d < needlepointsOpt.size(); d++)
                {
                    cv::Point3d currentPoint_Point3d;

                    nHp.at<double>(0,0) = needlepointsOpt[d].x;
                    nHp.at<double>(1,0) = needlepointsOpt[d].y;
                    nHp.at<double>(2,0) = needlepointsOpt[d].z;
                    nHp.at<double>(3,0) = 1;

                    tHp =  tHn * nHn_ * nHp;

                    currentPoint_Point3d.x = tHp.at<double>(0,0);// /tHp.at<double>(3,0);
                    currentPoint_Point3d.y = tHp.at<double>(1,0);// /tHp.at<double>(3,0);
                    currentPoint_Point3d.z = tHp.at<double>(2,0);// /tHp.at<double>(3,0);

                    newPoseNeedle3DModel.push_back(currentPoint_Point3d);
                }

                newPosesNeedle3DModel.push_back(newPoseNeedle3DModel);
                newPoses.push_back(tHn * nHn_);
            }

    for (int i=0; i<newPosesNeedle3DModel.size(); i++)
    {
        printNeedlePntsGlobalImage(frameL, frameR, newPosesNeedle3DModel[i], toolIndx, r*i/newPosesNeedle3DModel.size(), 255-g*i/newPosesNeedle3DModel.size(), 255-b*i/newPosesNeedle3DModel.size());
    }




}

