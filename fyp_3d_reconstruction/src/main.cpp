#include <ros/ros.h>
#include <iostream>
#include "global.h"
#include "readInImages.h"
#include "surf.h"
#include "viz.h"
#include "wrapper.h"
#include "bundle.h"

#include <opencv2/core.hpp>

// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

using namespace std; 
using namespace cv;
using namespace Eigen;

/***********************************************/

/*                    Main                     */

/***********************************************/

int main(int argc, char** argv)
{
    /* initiate ros node and publisher */

    ros::init(argc, argv, "Reconstruction");
    ros::NodeHandle nh;
    ros::Publisher pub_pts = nh.advertise<visualization_msgs::Marker>("recon_pts", 500);
    ros::Publisher pub_cam = nh.advertise<visualization_msgs::Marker>("recon_cam", 10);

    /* Variables */

    Mat img[N];  								// original images
    Mat img_undist[N];  						// undistorted images
    projMatr0 = camIntrinsic * Rt_init;

    //    /* --------------------- Specifies executable usage --------------------- */
    //    if(argc != 3)
    //    {
    //        cout << "Usage: ./fyp_3d_reconstruction <mode> [directory]" << endl;
    //        cout << "       <mode> : images" << endl;
    //        cout << "       [directory] : path to image folder. Do NOT add '/' at the end.
    //    						Folder should only contain image files" << endl;
    //        return -1;
    //    }


    /* ------------ In "images" mode - reading images from folder ----------- */

    //    if (strcmp(argv[1], "images") == 0)  // if argv[1] is "images"
    //    {
    //        cout << "Mode selected: images" << endl;


    /* Get folder path */ 
    const string folder = "/home/xsunaf/FYP/FYP_3D_Reconstruction/photos/test-4";
    cout << "Folder path: " << folder << endl;

    /* Read in images from folder path */
    readInImages(img, folder, m);
    /* Show Image size */
    cout << "Picture Height: " << img[0].rows << endl << "Picture Width: " << img[0].cols << endl;
    //    }   


    /* Variables */
    vector<KeyPoint> keypoints[m];
    vector< DMatch > good_matches[m-1];  				// matches for drawing
    vector< Point2f > points[m-1], pointsCompare[m-1],mask3D[m];  	// coordinates of matched points
    Mat img_matches[m-1];  								// img with matches for display
    Mat E;  										// essential matrix
    Mat mask; 										// mask for inliers after RANSAC
    Mat R[m-1],_R[m-1],t[m-1],_t[m-1]; 								// rotation matrices, cam translations
    Quaterniond q[m-1]; 								// orientation quaternions
    n_matches = new int[m-1];
    mid = m/2;


    /* ------------------------- 3D Reconstruction -------------------------- */

    /* Undistortion with cam parameters */
    undistort(img, img_undist, camIntrinsic, dist);

    /* SURF detector for features and matching*/
    surf(img_undist, good_matches,keypoints, points, pointsCompare, 400, img_matches);

    /* find essential matrix using five-point algorithm with RANSAC */
    E= findEssentialMat(points[0], pointsCompare[0], camIntrinsic, RANSAC, 0.999, 1.0, mask);

    /* use cheirality check to obtain R, t */
    recoverPose(E, points[0], pointsCompare[0], camIntrinsic, R[0], t[0], mask);
    t[0] = 0.552 * t[0];

    // mask out wrong 2d points
    vector< Point2f > pointsx,pointsComparex;
    for(int i=0;i<points[0].size();i++){
    	if (mask.at<uchar>(i,0) != 0){
    		pointsx.push_back(points[0][i]);
    		pointsComparex.push_back(pointsCompare[0][i]);
    	}
    }
    std::cout<<"first 2 images mask result: "<<points[0].size()<<"	"<<pointsx.size()<<std::endl;


    /* draw RANSAC inliers in red */
/*        for (int i=0; i<(m-1);i++){
            for (int j=0; j<n_matches[i];j++){
                if (mask.at<uchar>(j,0) != 0)
                    circle(img_matches[i], points[i][j], 10, Scalar(0,0,255), 3);
            }
        }
*/




    /* Triangulation */
    Mat points4D;   // homogeneous point world coordinates
    //@brief Reconstructs points by triangulation.
    //Keep in mind that all input data should be of float type in order for this function to work.

    triangulate_init(R[0],t[0],pointsx,pointsComparex,points4D,mask3D);

    // 4d to 3d
    Mat points3D(3,points4D.cols,CV_32F);
    for (int i=0; i<points4D.cols; i++)
    {
    	float x = points4D.at<float>(3,i);
        points3D.at<float>(0,i) = points4D.at<float>(0,i) / x;
        points3D.at<float>(1,i) = points4D.at<float>(1,i) / x;
        points3D.at<float>(2,i) = points4D.at<float>(2,i) / x;
   }

    std::cout<<"number of 3D points by first 2 images: "<<points3D.cols<<std::endl;
    std::cout<< "-------------------------------" << std::endl;

    /* ---------------------------------------------------------------------------------------*/


    // get R and t of the newest cam
    PnP(good_matches,2,keypoints,R,t,points,pointsCompare,mask3D,img_matches[1]);
    std::cout<< "-------------------------------" << std::endl;

    // take deducted matches and triangulate
    add_Points(R,t,points,pointsCompare,points3D,2,mask3D,img_matches[1]);

    std::cout<<points3D.cols<<std::endl;

    // get R and t of the newest cam
    //PnP(good_matches,3,keypoints,R,t,points,pointsCompare,mask3D,img_matches[1]);
    //std::cout<< "-------------------------------" << std::endl;

    // take deducted matches and triangulate
   // add_Points(R,t,points,pointsCompare,points3D,3,mask3D,img_matches[1]);

    //std::cout<<points3D.cols<<std::endl;

    /* ------------------- FOR BUNDLE ADJUSTMENT TEST -------------------- */

/*
    Mat RM[m_cam], TV[m_cam];  // camera R, T relative to the first cam
    RM[0] = R[0];      // first cam serve as the world frame
    RM[1] = R_21;
    TV[0] = t[0];
    TV[1] = t_21;
*/

    vector< vector <Point2f> > vec;  // rows - cams, cols - 3Dpoints; in pixel frame
    vec.resize(m);
    vector<Point3f> pts3D;
    int n_pts = points3D.cols;
    for (int i=0;i<n_pts;i++)
    {
        //if (mask.at<uchar>(i,0) != 0)
        //{
            vec[0].push_back(mask3D[0][i]);
            vec[1].push_back(mask3D[1][i]);
            vec[2].push_back(mask3D[2][i]);
            Point3f p;
            p.x = points3D.at<float>(0,i);
            p.y = points3D.at<float>(1,i);
            p.z = points3D.at<float>(2,i);
            pts3D.push_back(p);
        //}
    }

    //int n_pts = points3D.size();
    Mat Rx[m],tx[m];
    for(int i=0;i<m;i++){
    	if(i==0){
    	Rx[0] = R0;
    	tx[0] = t0;
    	}else{
    		Rx[i] = R[i-1];
    		tx[i] = t[i-1];
    	}
    }
    bundle(Rx, tx, pts3D, vec, m, n_pts);

    /* ----------------------- Visualization with RViz --------------------------*/
    invertpose(R,t,_R,_t);
    /* transform rotation matrix to quaternion */
    r2q(_R,q0,q);
    while (ros::ok()){
        viz(points3D,pub_pts,pub_cam,_t,q,m);
    }

    delete n_matches;
    return 0;
}

