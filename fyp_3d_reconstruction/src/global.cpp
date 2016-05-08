#include "global.h"
using namespace cv;

/* max number of images */
const int N = 30;

/* camera matrix */
//const Mat camIntrinsic = (Mat_<float>(3,3) << 4269.4,  0,  2629.6,    // [ Fx,  0, cx ]
//                                               0,  4269.5,  1728.6,    // [  0, Fy, cy ]
//                                               0,       0,       1);   // [  0,  0,  1 ]

/* camera matrix of P3 */
const Mat camIntrinsic = (Mat_<float>(3,3) << 2349.2,  0,  2005.8,    // [ Fx,  0, cx ]
                                               0,  2352.2,  1497.7,    // [  0, Fy, cy ]
                                               0,       0,       1);   // [  0,  0,  1 ]

/* distortion coefficients */
//const Mat dist = (Mat_<float>(1,4) << -0.1874, 0.1595, 0, 0);   // [k1, k2, p1, p2]

/* distortion coefficients of P3 */
const Mat dist = (Mat_<float>(1,4) << -0.0109, 0.0158, 0, 0);   // [k1, k2, p1, p2]

/* initialization of the first camera R,t,q */
const cv::Mat Rt_init = (cv::Mat_<float>(3,4) << 1,0,0,0,0,1,0,0,0,0,1,0) ;
cv::Mat projMatr0(3,4,CV_32F);

int m;  // number of cameras
int n;  // number of points

int *n_matches; 	// number of matched points










