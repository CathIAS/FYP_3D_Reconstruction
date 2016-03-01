#include "global.h"

/* max number of images */
const int N = 30;
/* camera matrix */
const Mat camIntrinsic = (Mat_<float>(3,3) << 4269.4,  0,  2629.6,    // [ Fx,  0, cx ]
                                               0,  4269.5,  1728.6,    // [  0, Fy, cy ]
                                               0,       0,       1);   // [  0,  0,  1 ]
/* distortion coefficients */
const Mat dist = (Mat_<float>(1,4) << -0.1874, 0.1595, 0, 0);   // [k1, k2, p1, p2]
