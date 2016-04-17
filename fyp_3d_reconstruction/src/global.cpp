#include "global.h"
using namespace cv;

/* max number of images */
const int N = 30;
/* camera matrix */
const Mat camIntrinsic = (Mat_<float>(3,3) << 4269.4,  0,  2629.6,    // [ Fx,  0, cx ]
                                               0,  4269.5,  1728.6,    // [  0, Fy, cy ]
                                               0,       0,       1);   // [  0,  0,  1 ]
/* distortion coefficients */
const Mat dist = (Mat_<float>(1,4) << -0.1874, 0.1595, 0, 0);   // [k1, k2, p1, p2]

/* initialization of the first camera R,t,q */
const cv::Mat R0 = (Mat_<float>(3,3) << 1,0,0,0,1,0,0,0,1);
const cv::Mat t0 = (Mat_<float>(3,1) << 0,0,0);
const cv::Mat Rt_init = (cv::Mat_<float>(3,4) << 1,0,0,0,0,1,0,0,0,0,1,0) ;
cv::Mat projMatr0(3,4,CV_32F);

Eigen::Quaterniond q0;

int m;  // number of cameras

int *n_matches; 	// number of matched points

int mid;










