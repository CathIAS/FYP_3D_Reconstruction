/*
 * wrapper.cpp
 *
 *  Created on: Mar 3, 2016
 *      Author: xsunaf
 */
#include "wrapper.h"

//function wrapper for array input
void undistort(const cv::Mat img[], cv::Mat img_undist[], const cv::Mat cameraMatrix, const cv::Mat distCoeffs){
    for (int i=0; i<m; i++){
        undistort(img[i], img_undist[i], camIntrinsic, dist);
    }
}

void findEssentialMat(const std::vector< cv::Point2f > points_1[],const std::vector< cv::Point2f > points_2[],
						const cv::Mat cameraMatrix,int method ,double prob,
						double threshold,cv::Mat mask[], cv::Mat E[]){
	for(int i=0;i<(m-1);i++){
	 E[i]=findEssentialMat(points_1[i], points_2[i], camIntrinsic, method, prob, threshold, mask[i]);
	    std::cout << "Essential Matrix: " << std::endl << " " << E[i] << std::endl;
	}
}

void recoverPose(const cv::Mat E[], const std::vector< cv::Point2f > points_1[], const std::vector< cv::Point2f > points_2[],
				const cv::Mat cameraMatrix, cv::Mat R[], cv::Mat t[],cv::Mat mask[]){
	for(int i=0;i<(m-1);i++){
	recoverPose(E[i], points_1[i], points_2[i], cameraMatrix, R[i], t[i], mask[i]);
	}
}

void r2q( cv::Mat R[],Eigen::Quaternionf q0,Eigen::Quaternionf q[] ){

	Eigen::Matrix3f R_eigen;
	cv::cv2eigen(R0,R_eigen);
	q0 = R_eigen;
    q0.normalize();
	for(int i=0;i<(m-1);i++){
    cv::cv2eigen(R[i],R_eigen);
    q[i]=R_eigen;
    q[i].normalize();
	}
}

void triangulatePoints( const cv::Mat R[],const cv::Mat t[],const std::vector< cv::Point2f > points[],
						const std::vector< cv::Point2f > pointsCompare[],cv::Mat points4D[] ){

	cv::Mat Rt_init = (cv::Mat_<double>(3,4) << 1,0,0,0,0,1,0,0,0,0,1,0) ; // identity Rt

	/* calculate projection matrices */
    cv::Mat projMatr0(3,4,CV_64F);  // projection matrices
    projMatr0 = camIntrinsic * Rt_init;  // ProjMat = K * Rt

    for(int i=0;i<(m-1);i++){
    	cv::Mat projMatr(3,4,CV_64F);

    	cv::Mat Rt(3,4,CV_64F);  // Rt = [R | t]
    	hconcat(R[i], t[i], Rt);  // Rt concatenate
    	projMatr = camIntrinsic * Rt;

    	points4D[i]=cv::Mat_<double>(4,n_matches[i]);
    	cv::triangulatePoints(projMatr0, projMatr, points[i], pointsCompare[i], points4D[i]);
    }
}

