#ifndef BUNDLE_H
#define BUNDLE_H

#include "global.h"
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

Point2f pix2img(Point2f pt_pix);

void bundle(cv::Mat R[], cv::Mat T[], std::vector<Point3f> pts3, std::vector< std::vector<cv::Point2f> > z, int m_cam, int n_pts); 

// Cam - K: camera matrix, R[]: rotation matrices, T[]: translations
// Points - points3D (vector)
// Relation - points2D (vector of vector), (-1,-1) if cannot see from that cam
// m_cam - number of cameras
// n_pts - number of 3D points

#endif /* BUNDLE_H */
