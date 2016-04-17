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
        int inliers = recoverPose(E[i], points_1[i], points_2[i], cameraMatrix, R[i], t[i], mask[i]);
        std::cout << "Number of inliers in image " << i << " is " << inliers << std::endl;
    }
}

void r2q( cv::Mat _R[] ,Eigen::Quaterniond& q0,Eigen::Quaterniond q[] ){

    Eigen::Matrix3d R_eigen;
    cv::cv2eigen(R0,R_eigen);
    q0 = R_eigen;
    q0.normalize();
    std::cout << "Quaternion0: "<< std::endl << "w: "<<q0.w()<<std::endl<<"vector: "<< std::endl<<q0.vec() << std::endl;
    for(int i=0;i<(m-1);i++){
        cv::cv2eigen(_R[i],R_eigen);
        q[i]=R_eigen;
        q[i].normalize();
        std::cout << "Quaternion"<<i+1<<": "<< std::endl << "w: "<<q[i].w()<<std::endl<<"vector: "<< std::endl<<q[i].vec() << std::endl;
    }
}

void triangulate_Points( cv::Mat R[], cv::Mat t[],const std::vector< cv::Point2f > points_1[],
        const std::vector< cv::Point2f > points_2[],cv::Mat& points4D){

    //cv::Mat Rt_init = (cv::Mat_<double>(3,4) << 1,0,0,0,0,1,0,0,0,0,1,0) ; // identity Rt

    /* calculate projection matrices */
    //cv::Mat projMatr0(3,4,CV_32F);  // projection matrices
    //projMatr0 = camIntrinsic * Rt_init;  // ProjMat = K * Rt

        cv::Mat projMatr(3,4,CV_32F);

        cv::Mat Rt(3,4,CV_32F);  // Rt = [R | t]
        R[0].convertTo(R[0],CV_32F);
        t[0].convertTo(t[0],CV_32F);
        hconcat(R[0], t[0], Rt);  // Rt concatenate
        std::cout<<Rt<<std::endl;
        projMatr = camIntrinsic * Rt;

        points4D=cv::Mat_<float>(4,points_1[0].size());
        cv::triangulatePoints(projMatr0, projMatr, points_1[0], points_2[0], points4D);

}

void add_Points( cv::Mat R[],cv::Mat t[],const std::vector< cv::Point2f > points_1[],
        const std::vector< cv::Point2f > points_2[],cv::Mat& points3D,const int add){
    /* calculate projection matrices */
    //cv::Mat projMatr0(3,4,CV_32F);  // projection matrices
    //projMatr0 = camIntrinsic * Rt_init;  // ProjMat = K * Rt

        cv::Mat projMatr(3,4,CV_32F);

        cv::Mat Rt(3,4,CV_32F);  // Rt = [R | t]
        R[add-1].convertTo(R[add-1],CV_32F);
        t[add-1].convertTo(t[add-1],CV_32F);
        hconcat(R[add-1], t[add-1], Rt);  // Rt concatenate

        projMatr = camIntrinsic * Rt;
       // std::cout<<Rt<<std::endl<<projMatr.rows<<projMatr.cols<<std::endl;
        cv::Mat points4Dtemp=cv::Mat_<float>(4,points_1[add-1].size());
        cv::triangulatePoints(projMatr0, projMatr, points_1[add-1], points_2[add-1], points4Dtemp);

        cv::Mat points3Dtemp(3,points_1[add-1].size(),CV_32F);

        for (int i=0; i<points_1[add-1].size(); i++)
        {
        	float x = points4Dtemp.at<float>(3,i);
            points3Dtemp.at<float>(0,i) = points4Dtemp.at<float>(0,i) / x;
            points3Dtemp.at<float>(1,i) = points4Dtemp.at<float>(1,i) / x;
            points3Dtemp.at<float>(2,i) = points4Dtemp.at<float>(2,i) / x;
       }
        hconcat(points3D,points3Dtemp,points3D);
}

void PnP(const std::vector< cv::DMatch > good_matches[],const int add,const std::vector<cv::KeyPoint> keypoints[],
		cv::Mat R[], cv::Mat t[],std::vector< cv::Point2f > points_1[], std::vector< cv::Point2f > points_2[]){

	std::vector<int> indicator(5,-1);
	std::vector<cv::Point2f> points1,points2,points3;
	for(int i = 0; i < good_matches[add-2].size();i++){
		if(good_matches[add-2][i].trainIdx>=indicator.size()){
			indicator.resize(good_matches[add-2][i].trainIdx+1,-1);
			indicator[good_matches[add-2][i].trainIdx] = i;
		}
		else{
			indicator[good_matches[add-2][i].trainIdx] = i;
		}
	}
	std::cout<<points_1[add-1].size()<<"	"<<points_2[add-1].size()<<std::endl;
	for(int i = 0; i < good_matches[add-1].size();i++){
		if(good_matches[add-1][i].queryIdx<indicator.size()){
		int ind = good_matches[add-1][i].queryIdx;
		if(indicator[ind]!=-1){
			cv::Point2f temppoint = keypoints[add-2][good_matches[add-2][indicator[ind]].queryIdx].pt ;
			points1.push_back(temppoint);
			//points_1[add-2].erase( std::remove( points_1[add-2].begin(), points_1[add-2].end(), temppoint ), points_1[add-2].end() );



			temppoint =keypoints[add-1][good_matches[add-2][indicator[ind]].trainIdx].pt;
			std::vector<cv::Point2f>::iterator p = std::find(points_1[add-1].begin(), points_1[add-1].end(), temppoint);
			points2.push_back(temppoint);
			if(p!=points_1[add-1].end()){
				points_1[add-1].erase(p);
			}
			//points_1[add-1].erase( std::remove( points_1[add-1].begin(), points_1[add-1].end(), temppoint ), points_1[add-1].end() );

			temppoint = keypoints[add][good_matches[add-1][i].trainIdx].pt;
			std::vector<cv::Point2f>::iterator t = std::find(points_2[add-1].begin(), points_2[add-1].end(), temppoint);
			points3.push_back(temppoint);
			if(t!=points_2[add-1].end()){
				points_2[add-1].erase(t);
			}
			//points_2[add-1].erase( std::remove( points_2[add-1].begin(), points_2[add-1].end(), temppoint ), points_2[add-1].end() );

		}

	}
	}
	std::cout<<points1.size()<<"	"<<points2.size()<<"	"<<points3.size()<<std::endl;
	std::cout<<points_1[add-1].size()<<"	"<<points_2[add-1].size()<<std::endl;

	//prepare 3d points for pnp
			//cv::Mat projMatr0(3,4,CV_64F);  // projection matrices
			//projMatr0 = camIntrinsic * Rt_init;  // ProjMat = K * Rt

	        cv::Mat projMatr(3,4,CV_32F);

	        cv::Mat Rt(3,4,CV_32F);  // Rt = [R | t]
	        R[add-2].convertTo(R[add-2],CV_32F);
	        t[add-2].convertTo(t[add-2],CV_32F);
	        hconcat(R[add-2], t[add-2], Rt);  // Rt concatenate
	        projMatr = camIntrinsic * Rt;
	        std::cout<<Rt<<std::endl;
	        cv::Mat points4Dtemp=cv::Mat_<float>(4,points1.size());
	        cv::triangulatePoints(projMatr0, projMatr, points1, points2, points4Dtemp);

	        std::vector<cv::Point3f> points3Dtemp;
	        for (int i=0; i < points1.size(); i++)
	        {
	        	float x = points4Dtemp.at<float>(3,i);
	            //points3Dtemp.at<float>(0,i) = points4Dtemp.at<float>(0,i) / x;
	            //points3Dtemp.at<float>(1,i) = points4Dtemp.at<float>(1,i) / x;
	            //points3Dtemp.at<float>(2,i) = points4Dtemp.at<float>(2,i) / x;
	        	cv::Point3f p;
	        	p.x = points4Dtemp.at<float>(0,i) / x;
	        	p.y = points4Dtemp.at<float>(1,i) / x;
	        	p.z = points4Dtemp.at<float>(2,i) / x;
	        	points3Dtemp.push_back(p);
	       }
	        //std::cout<<points3Dtemp.rows<<"		"<<std::endl;
	        //std::cout<<"6666666666666666666666666666666"<<std::endl;
	        cv::Mat Rv;
	        solvePnPRansac(points3Dtemp,points3,camIntrinsic,dist,Rv,t[add-1]);
	        Rodrigues(Rv,R[add-1]);
	        //std::cout<<"5555555555555555555555555555555"<<std::endl;

}
void invertpose(const cv::Mat R[], cv::Mat t[],cv:: Mat _R[], cv::Mat _t[] ){
    std::cout << "Rotation Matrix0 : " << std::endl << " " << R0 << std::endl;
    std::cout << "Translation0 : " << std::endl << " " << t0 << std::endl;
	for(int i = 0;i< (m-1);i++){
	    // Obtain inverse:
	    _R[i] = R[i].inv();
	    _t[i] = -1 * _R[i] * t[i];
	    std::cout << "Rotation Matrix"<<i+1<< ": " << std::endl << " " << R[i] << std::endl;
	    std::cout << "__Rotation Matrix"<<i+1<< ": " << std::endl << " " << _R[i] << std::endl;
	    std::cout << "Translation"<<i+1<< ": " << std::endl << " " << t[i] << std::endl;
	    std::cout << "__Translation"<<i+1<< ": " << std::endl << " " << _t[i] << std::endl;
	}

}

