#ifndef SURF_H
#define SURF_H

#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgproc.hpp"

void surf(const cv::Mat img_undist[],std::vector< cv::DMatch > good_matches,std::vector< cv::Point2f > points_1,std::vector< cv::Point2f > points_2,const int hessian,const int idx_1,const int idx_2,cv::Mat img_matches);
void surf(const cv::Mat img_undist[],std::vector< cv::DMatch > good_matches[],std::vector< cv::Point2f > points_1[],std::vector< cv::Point2f > points_2[],const int hessian,cv::Mat img_matches[]);
#endif /* SURF_H */
