#ifndef GLOBAL_H
#define GLOBAL_H

#include "opencv2/core/core.hpp"

using namespace cv;

/* max number of images */
extern const int N;

/* camera matrix */
extern const Mat camIntrinsic;

/* distortion coefficients */
extern const Mat dist;

#endif /* GLOBAL_H */
