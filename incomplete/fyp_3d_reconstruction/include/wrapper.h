/*
 * wrapper.h
 *
 *  Created on: Mar 3, 2016
 *      Author: xsunaf
 */

#ifndef FYP_3D_RECONSTRUCTION_INCLUDE_WRAPPER_H_
#define FYP_3D_RECONSTRUCTION_INCLUDE_WRAPPER_H_
#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "opencv2/core.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include "global.h"

void undistort(const cv::Mat img[], cv::Mat img_undist[], const cv::Mat cameraMatrix, const cv::Mat distCoeffs);


void r2q(cv::Mat R[],Eigen::Quaterniond& q0,Eigen::Quaterniond q[]);

void triangulate_init(cv::Mat R, cv::Mat t,const std::vector< cv::Point2f > points_1,
						const std::vector< cv::Point2f > points_2,cv::Mat& points4D,std::vector< cv::Point2f > mask3D[]);

void add_Points( cv::Mat R[], cv::Mat t[],const std::vector< cv::Point2f > points_1[],
        const std::vector< cv::Point2f > points_2[],cv::Mat& points3D,const int add,std::vector< cv::Point2f > mask3D[],cv::Mat& img_matches);

void PnP(const std::vector< cv::DMatch > good_matches[],const int q,const std::vector<cv::KeyPoint> keypoints[], cv::Mat R[],
		cv::Mat t[],std::vector< cv::Point2f > points_1[], std::vector< cv::Point2f > points_2[],std::vector< cv::Point2f > mask3D[],cv::Mat& img_matches);

void invertpose(const cv::Mat R[],cv::Mat t[],cv:: Mat _R[], cv::Mat _t[] );

#endif /* FYP_3D_RECONSTRUCTION_INCLUDE_WRAPPER_H_ */
