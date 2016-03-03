#include "global.h"
using namespace cv;

/* max number of images */
const int N = 30;
/* camera matrix */
const Mat camIntrinsic = (Mat_<double>(3,3) << 4269.4,  0,  2629.6,    // [ Fx,  0, cx ]
                                               0,  4269.5,  1728.6,    // [  0, Fy, cy ]
                                               0,       0,       1);   // [  0,  0,  1 ]
/* distortion coefficients */
const Mat dist = (Mat_<double>(1,4) << -0.1874, 0.1595, 0, 0);   // [k1, k2, p1, p2]

/* initialization of the first camera R,t,q */
 cv::Mat R0 = (Mat_<double>(3,3) << 1,0,0,0,1,0,0,0,1);
 cv::Mat t0 = (Mat_<double>(3,1) << 0,0,0);

int m;

int *n_matches;  										// number of matched points

int mid;










