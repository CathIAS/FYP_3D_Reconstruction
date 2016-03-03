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
#include <opencv2/core/eigen.hpp>

#include "global.h"

void undistort(const cv::Mat img[], cv::Mat img_undist[], const cv::Mat cameraMatrix, const cv::Mat distCoeffs);

void findEssentialMat(const std::vector< cv::Point2f > points_1[],const std::vector< cv::Point2f > points_2[],
						const cv::Mat cameraMatrix, int method, double prob,
						double threshold, cv::Mat mask[], cv::Mat E[]);

void recoverPose(const cv::Mat E[], const std::vector< cv::Point2f > points_1[],const std::vector< cv::Point2f > points_2[],
				const cv::Mat cameraMatrix, cv::Mat R[], cv::Mat t[],cv::Mat mask[]);

void r2q(cv::Mat R[],Eigen::Quaternionf q0,Eigen::Quaternionf q[]);

void triangulatePoints( const cv::Mat R[],const cv::Mat t[],const std::vector< cv::Point2f > points[],
						const std::vector< cv::Point2f > pointsCompare[],cv::Mat points4D[] );

#endif /* FYP_3D_RECONSTRUCTION_INCLUDE_WRAPPER_H_ */
