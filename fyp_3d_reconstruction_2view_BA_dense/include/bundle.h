#ifndef BUNDLE_H
#define BUNDLE_H

#include "global.h"
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <iomanip>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <ctime>


// convert from pixel fraame to image frame using camera matrix
Point2f pix2img(Point2f pt_pix);

// projection function Jacobian 2x3
Eigen::MatrixXf cal_Jp( const Eigen::Matrix3f R, const Eigen::Vector3f T, const Eigen::Vector3f X);

// complete Jacobian of theta, translation and landmark coordinates 2x3
Eigen::MatrixXf cal_Jt( const Eigen::Matrix3f R, const Eigen::Vector3f T, const Eigen::Vector3f X);
Eigen::MatrixXf cal_Jr( const Eigen::Matrix3f R, const Eigen::Vector3f T, const Eigen::Vector3f X);
Eigen::MatrixXf cal_Jx( const Eigen::Matrix3f R, const Eigen::Vector3f T, const Eigen::Vector3f X);

// convert variables to Eigen library
void toEigen( const cv::Mat R[], const cv::Mat T[], const cv::Mat theta[], const std::vector<Point3f> pts3, Eigen::Matrix3f Rot[], Eigen::Vector3f Theta[], Eigen::Vector3f Tra[], Eigen::Vector3f P3[], int m, int n);

// convert variables to OpenCV library
void toCV( cv::Mat R[], cv::Mat T[], cv::Mat theta[], std::vector<Point3f>& pts3, const Eigen::Matrix3f Rot[], const Eigen::Vector3f Theta[], const Eigen::Vector3f Tra[], const Eigen::Vector3f P3[], int m, int n);

// get reprojections in image frame
std::vector< std::vector<Point2f> >  getReprojection( const Eigen::Vector3f Theta[], const Eigen::Vector3f Tra[], const Eigen::Vector3f P3[], int m, int n);

// get observations in image frame
std::vector< std::vector<Point2f> >  getObservation( const std::vector< std::vector<cv::Point2f> >& z, int m, int n);

// update error vector 
void updateError( const std::vector< std::vector<cv::Point2f> >& z, const std::vector< std::vector<Point2f> >& rep, const std::vector< std::vector<Point2f> >& z_img, const int m_cam, const int n_pts, Eigen::VectorXf& err);

// form Jacobian Matrix
void getJacobian( const std::vector< std::vector<cv::Point2f> >& z, const Eigen::Matrix3f Rot[], const Eigen::Vector3f Tra[], const Eigen::Vector3f P3[], int m_cam, int n_pts, Eigen::MatrixXf& J);

// increment delta x to all the variables
void increX( const Eigen::VectorXf& del_x, Eigen::Vector3f Theta[], Eigen::Matrix3f Rot[], Eigen::Vector3f Tra[], Eigen::Vector3f P3[], int m_cam, int n_pts);

// Bundle Adjustment main function
void bundle(cv::Mat R[], cv::Mat T[], std::vector<Point3f>& pts3, const std::vector< std::vector<cv::Point2f> >& z, int m_cam, int n_pts); 

// Cam - K: camera matrix, R[]: rotation matrices, T[]: translations
// Points - points3D (vector)
// Relation - points2D (vector of vector), (-1,-1) if cannot see from that cam
// m_cam - number of cameras
// n_pts - number of 3D points

#endif /* BUNDLE_H */
