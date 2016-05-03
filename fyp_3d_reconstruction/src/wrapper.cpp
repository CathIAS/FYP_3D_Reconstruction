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

void r2q( cv::Mat _R[] ,Eigen::Quaterniond q[] ){

    Eigen::Matrix3d R_eigen;
    for(int i=0;i<m;i++){
        cv::cv2eigen(_R[i],R_eigen);
        q[i]=R_eigen;
        q[i].normalize();
        std::cout << "Quaternion"<<i<<": "<< std::endl << "w: "<<q[i].w()<<std::endl<< std::endl<<q[i].vec() << std::endl;
    }
}

void triangulate_init( cv::Mat R, cv::Mat t,const std::vector< cv::Point2f > points_1,
        const std::vector< cv::Point2f > points_2,cv::Mat& points4D,std::vector< cv::Point2f > mask3D[]){

        cv::Mat projMatr(3,4,CV_32F);

        cv::Mat Rt(3,4,CV_32F);  // Rt = [R | t]
        R.convertTo(R,CV_32F);
        t.convertTo(t,CV_32F);
        hconcat(R, t, Rt);  // Rt concatenate
        projMatr = camIntrinsic * Rt;

        points4D=cv::Mat_<float>(4,points_1.size());
        cv::triangulatePoints(projMatr0, projMatr, points_1, points_2, points4D);
        cv::Point2f x(-1,-1);
        for(int i = 0;i<points4D.cols;i++){
        	mask3D[0].push_back(points_1[i]);
        	mask3D[1].push_back(points_2[i]);
        	mask3D[2].push_back(x);
        }

}

void add_Points( cv::Mat R[],cv::Mat t[],const std::vector< cv::Point2f > points_1[],
        const std::vector< cv::Point2f > points_2[],cv::Mat& points3D,const int add,std::vector< cv::Point2f > mask3D[],cv::Mat& img_matches){

	/* calculate projection matrices */

	cv::Mat mask;
	findEssentialMat(points_1[add-1], points_2[add-1], camIntrinsic, cv::RANSAC, 0.9, 1.0, mask);
    std::vector< cv::Point2f > pointsx,pointsComparex;
    for(int i=0;i<points_1[add-1].size();i++){
    	if (mask.at<uchar>(i,0) != 0){
    		pointsx.push_back(points_1[add-1][i]);
    		pointsComparex.push_back(points_2[add-1][i]);
    	}
    }
    std::cout<<"mask result: "<<points_1[add-1].size()<<"	"<<pointsx.size()<<std::endl;

  /*  for (int j=0; j<points_1[add-1].size();j++){
                    if (mask.at<uchar>(j,0) != 0)
                        circle(img_matches, points_1[add-1][j], 10, cv::Scalar(0,0,255), 3);
                }

    cv::namedWindow("Matches", CV_WINDOW_NORMAL);
    cv::imshow("Matches", img_matches);
    cv::waitKey();
*/

		cv::Mat projMatr1(3,4,CV_32F);
		cv::Mat Rt1(3,4,CV_32F);  // Rt = [R | t]
		R[add-2].convertTo(R[add-2],CV_32F);
		t[add-2].convertTo(t[add-2],CV_32F);
		hconcat(R[add-2], t[add-2], Rt1);  // Rt concatenate
		projMatr1 = camIntrinsic * Rt1;

        cv::Mat projMatr2(3,4,CV_32F);
        cv::Mat Rt(3,4,CV_32F);  // Rt = [R | t]
        R[add-1].convertTo(R[add-1],CV_32F);
        t[add-1].convertTo(t[add-1],CV_32F);
        hconcat(R[add-1], t[add-1], Rt);  // Rt concatenate
        projMatr2 = camIntrinsic * Rt;

       // std::cout<<Rt<<std::endl<<projMatr.rows<<projMatr.cols<<std::endl;
        cv::Mat points4Dtemp=cv::Mat_<float>(4,points_1[add-1].size());
        cv::triangulatePoints(projMatr1, projMatr2, pointsx, pointsComparex, points4Dtemp);
        cv::Point2f x(-1,-1);
        for(int i = 0;i<points4Dtemp.cols;i++){
        	mask3D[0].push_back(x);
        	mask3D[1].push_back(pointsx[i]);
        	mask3D[2].push_back(pointsComparex[i]);
        }
        cv::Mat points3Dtemp(3,pointsx.size(),CV_32F);

        for (int i=0; i<pointsx.size(); i++)
        {
        	float x = points4Dtemp.at<float>(3,i);
            points3Dtemp.at<float>(0,i) = points4Dtemp.at<float>(0,i) / x;
            points3Dtemp.at<float>(1,i) = points4Dtemp.at<float>(1,i) / x;
            points3Dtemp.at<float>(2,i) = points4Dtemp.at<float>(2,i) / x;
       }
        hconcat(points3D,points3Dtemp,points3D);
}

void PnP(const std::vector< cv::DMatch > good_matches[],const int add,const std::vector<cv::KeyPoint> keypoints[],
		cv::Mat R[], cv::Mat t[],std::vector< cv::Point2f > points_1[], std::vector< cv::Point2f > points_2[],std::vector< cv::Point2f > mask3D[],cv::Mat& img_matches){

	std::vector<int> indicator(5,-1);
	std::vector<cv::Point2f> points1,points2,points3;

	// image 1 & 2
	// write in indicator for last 2 images
	for(int i = 0; i < good_matches[add-2].size();i++){
		if(good_matches[add-2][i].trainIdx>=indicator.size()){
			indicator.resize(good_matches[add-2][i].trainIdx+1,-1);
			indicator[good_matches[add-2][i].trainIdx] = i;
		}
		else{
			indicator[good_matches[add-2][i].trainIdx] = i;
		}
	}

	// images 2 & 3
	std::cout<< "number of matches between "<<add+1<<" pic and last pic: "<<points_1[add-1].size()<<std::endl;   // how many matches initially

	// assign points1, points2, points3; find common matches and update mask3d and points_2
	for(int i = 0; i < good_matches[add-1].size();i++){
		if(good_matches[add-1][i].queryIdx<indicator.size()){
			int ind = good_matches[add-1][i].queryIdx;
			if(indicator[ind]!=-1){

				cv::Point2f temppoint = keypoints[add-2][good_matches[add-2][indicator[ind]].queryIdx].pt ;
				points1.push_back(temppoint);

				temppoint =keypoints[add-1][good_matches[add-2][indicator[ind]].trainIdx].pt;
				std::vector<cv::Point2f>::iterator p = std::find(points_1[add-1].begin(), points_1[add-1].end(), temppoint);
				std::vector<cv::Point2f>::iterator _p = std::find(mask3D[1].begin(), mask3D[1].end(), temppoint);
				points2.push_back(temppoint);

				temppoint = keypoints[add][good_matches[add-1][i].trainIdx].pt;
				std::vector<cv::Point2f>::iterator t = std::find(points_2[add-1].begin(), points_2[add-1].end(), temppoint);
				points3.push_back(temppoint);

				// update mask3d info 3rd cam
				if(_p!=mask3D[1].end()){
					int nPosition = distance (mask3D[1].begin(), _p);
					mask3D[2][nPosition] = temppoint;
				}

				// delete common matches across 3 pics from points_1 and points_2
				if(p!=points_1[add-1].end()){
					int nPosition = distance (points_1[add-1].begin(), p);
					points_1[add-1].erase(p);
					points_2[add-1].erase(points_2[add-1].begin()+nPosition);
				}
				//if(t!=points_2[add-1].end()){
				//	points_2[add-1].erase(t);
				//}
			}
		}
	}
/*    for (int j=0; j<points_1[add-1].size();j++){

                        circle(img_matches, points_1[add-1][j], 10, cv::Scalar(0,0,255), 3);
                }

    cv::namedWindow("Matches", CV_WINDOW_NORMAL);
    cv::imshow("Matches", img_matches);
    cv::waitKey();
*/
	std::cout<<"number of common matches found across 3 pics: "<<points1.size()<<std::endl;
	std::cout<<"number of matches between "<<add+1<<" pic and last pic after deduction: "<<points_1[add-1].size()<<"	"<<points_2[add-1].size()<<std::endl;

	// reconstruct 3d points of common matches from last pics for pnp

		cv::Mat projMatr1(3,4,CV_32F);
		if(add==2){
			projMatr1=projMatr0;
		}
		else{
		cv::Mat Rt1(3,4,CV_32F);  // Rt = [R | t]
		R[add-3].convertTo(R[add-3],CV_32F);
		t[add-3].convertTo(t[add-3],CV_32F);
		hconcat(R[add-3], t[add-3], Rt1);  // Rt concatenate

		projMatr1 = camIntrinsic * Rt1;
		}
	        cv::Mat projMatr2(3,4,CV_32F);
	        cv::Mat Rt(3,4,CV_32F);  // Rt = [R | t]
	        R[add-2].convertTo(R[add-2],CV_32F);
	        t[add-2].convertTo(t[add-2],CV_32F);
	        hconcat(R[add-2], t[add-2], Rt);  // Rt concatenate
	        projMatr2 = camIntrinsic * Rt;
//	        std::cout<<Rt<<std::endl;

	        cv::Mat points4Dtemp=cv::Mat_<float>(4,points1.size());

/*
	        cv::Mat mask1,mask2;
	        cv::findEssentialMat(points1, points2, camIntrinsic, cv::RANSAC, 0.999, 1.0, mask1);
	        std::vector< cv::Point2f > points1x, points2x, points3x;
	            for(int i=0;i<points1.size();i++){
	            	if (mask1.at<uchar>(i,0) != 0){
	            		points1x.push_back(points1[i]);
	            		points2x.push_back(points2[i]);
	            		points3x.push_back(points3[i]);
	            	}
	        }

		    cv::findEssentialMat(points2x, points3x, camIntrinsic, cv::RANSAC, 0.999, 1.0, mask2);
		    std::vector< cv::Point2f > points1xx, points2xx, points3xx;
		        for(int i=0;i<points1x.size();i++){
		        	if (mask2.at<uchar>(i,0) != 0){
		        		points1xx.push_back(points1x[i]);
		        		points2xx.push_back(points2x[i]);
		         		points3xx.push_back(points3x[i]);
		          	}
	        }

	        std::cout<<"number of common matches after ransac: "<<points1xx.size()<<std::endl;
*/


	        cv::triangulatePoints(projMatr1, projMatr2, points1, points2, points4Dtemp);
	        std::vector<cv::Point3f> points3Dtemp;
	        for (int i=0; i < points1.size(); i++)
	        {
	        	float x = points4Dtemp.at<float>(3,i);
	        	cv::Point3f p;
	        	p.x = points4Dtemp.at<float>(0,i) / x;
	        	p.y = points4Dtemp.at<float>(1,i) / x;
	        	p.z = points4Dtemp.at<float>(2,i) / x;
	        	points3Dtemp.push_back(p);
	       }

	        //std::cout<<points3Dtemp.rows<<"		"<<std::endl;
	        //std::cout<<"6666666666666666666666666666666"<<std::endl;

	    // PnP to get R and t of current pic

	        cv::Mat Rv;
	        cv::Mat inlier;
	        solvePnPRansac(points3Dtemp,points3,camIntrinsic,dist,Rv,t[add-1],false,100,1.0,0.999,inlier);
	        int n_inl = countNonZero(inlier);
	        std::cout<<"number of inliers in PnP: "<<n_inl<<std::endl;

	        Rodrigues(Rv,R[add-1]);
	        //std::cout<<"5555555555555555555555555555555"<<std::endl;
//	        std::cout<<"R after pnp: "<<R[add-1]<<std::endl;
//	        std::cout<<"t after pnp: "<<t[add-1]<<std::endl;

}


void invertpose(const cv::Mat R[], cv::Mat t[],cv:: Mat _R[], cv::Mat _t[] ){
//    std::cout << "Rotation Matrix0 : " << std::endl << " " << R0 << std::endl;
//    std::cout << "Translation0 : " << std::endl << " " << t0 << std::endl;
	for(int i = 0;i<m;i++){
	    // Obtain inverse:
	    _R[i] = R[i].inv();
	    _t[i] = -1 * _R[i] * t[i];
	    std::cout << "Rotation Matrix"<<i<< ": " << std::endl << " " << R[i] << std::endl;
//	    std::cout << "__Rotation Matrix"<<i+1<< ": " << std::endl << " " << _R[i] << std::endl;
	    std::cout << "Translation"<<i<< ": " << std::endl << " " << t[i] << std::endl;
//	    std::cout << "__Translation"<<i+1<< ": " << std::endl << " " << _t[i] << std::endl;
	}

}

