#include "global.h"

/* max number of images */
const int n = 30;
/* camera matrix */
const Mat camIntrinsic = (Mat_<double>(3,3) << 2797.1,  0,  1602.1,    // [ Fx,  0, cx ]
                                               0,  2793.7,  1197.9,    // [  0, Fy, cy ]
                                               0,       0,       1);   // [  0,  0,  1 ]
/* distortion coefficients */
const Mat dist = (Mat_<double>(1,4) << 0.0949, -0.1358, 0, 0);   // [k1, k2, p1, p2]
