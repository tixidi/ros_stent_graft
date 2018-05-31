#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include <cv.h>

extern std::string CAMERA_CALIB_DIR;
extern std::string SRC_FILES_DIR;

extern cv::Mat M1, D1, mx1, my1, P1;
extern cv::Mat M2, D2, mx2, my2, P2;
