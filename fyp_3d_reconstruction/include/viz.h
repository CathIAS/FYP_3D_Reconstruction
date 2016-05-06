/*
 * viz.hpp
 *
 *  Created on: Mar 3, 2016
 *      Author: xsunaf
 */

#ifndef FYP_3D_RECONSTRUCTION_INCLUDE_VIZ_H_
#define FYP_3D_RECONSTRUCTION_INCLUDE_VIZ_H_
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include "global.h"
#include <ros/ros.h>
#include "opencv2/core.hpp"
#include <opencv2/core/eigen.hpp>

void r2q(cv::Mat R[],Eigen::Quaterniond q[], int num);

void invertpose(const cv::Mat R[], const cv::Mat t[],cv:: Mat _R[], cv::Mat _t[], int num );

void viz(cv::Mat points3D,ros::Publisher& pub_pts,ros::Publisher& pub_cam, const cv::Mat t[], const cv::Mat R[], Eigen::Quaterniond q[],int num);

#endif /* FYP_3D_RECONSTRUCTION_INCLUDE_VIZ_H_ */
