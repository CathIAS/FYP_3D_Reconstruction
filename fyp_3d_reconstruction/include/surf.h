#ifndef SURF_H
#define SURF_H

#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

void surf(Mat img_undist[],vector< DMatch >& good_matches);

#endif /* SURF_H */
