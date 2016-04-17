#ifndef GLOBAL_H
#define GLOBAL_H
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "opencv2/core/core.hpp"
/* max number of images */
extern const int N;

/* camera matrix */
extern const cv::Mat camIntrinsic;

/* distortion coefficients */
extern const cv::Mat dist;
extern const cv::Mat R0, t0;
extern const cv::Mat Rt_init;
extern cv::Mat projMatr0;
extern Eigen::Quaterniond q0;
extern int m;  								// number of images loaded
extern int *n_matches;  										// number of matched points
extern int mid;


#endif /* GLOBAL_H */
