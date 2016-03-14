/*
 * viz.hpp
 *
 *  Created on: Mar 3, 2016
 *      Author: xsunaf
 */

#ifndef FYP_3D_RECONSTRUCTION_INCLUDE_VIZ_H_
#define FYP_3D_RECONSTRUCTION_INCLUDE_VIZ_H_
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include "global.h"
#include <ros/ros.h>
void viz(const cv::Mat points4D,ros::Publisher& pub_pts,ros::Publisher& pub_cam,cv::Mat t[],cv::Mat mask[],Eigen::Quaterniond q[]);

#endif /* FYP_3D_RECONSTRUCTION_INCLUDE_VIZ_H_ */
