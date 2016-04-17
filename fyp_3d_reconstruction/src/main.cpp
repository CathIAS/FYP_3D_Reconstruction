#include <ros/ros.h>
#include <iostream>
#include "global.h"
#include "readInImages.h"
#include "surf.h"
#include "viz.h"
#include "wrapper.h"

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
    const string folder = "/home/xsunaf/FYP/FYP_3D_Reconstruction/photos/test-3";
    cout << "Folder path: " << folder << endl;

    /* Read in images from folder path */
    readInImages(img, folder, m);
    /* Show Image size */
    cout << "Picture Height: " << img[0].rows << endl << "Picture Width: " << img[0].cols << endl;
    //    }   


    /* Variables */
    vector<KeyPoint> keypoints[m];
    vector< DMatch > good_matches[m-1];  				// matches for drawing
    vector< Point2f > points[m-1], pointsCompare[m-1];  	// coordinates of matched points
    Mat img_matches[m-1];  								// img with matches for display
    Mat E[m-1];  										// essential matrix
    Mat mask[m-1]; 										// mask for inliers after RANSAC
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
    findEssentialMat(points, pointsCompare, camIntrinsic, RANSAC, 0.999, 1.0, mask, E);

    /* use cheirality check to obtain R, t */
    recoverPose(E[0], points[0], pointsCompare[0], camIntrinsic, R[0], t[0], mask[0]);
    t[0] = 0.552 * t[0];


    /* draw RANSAC inliers in red */
    for (int i=0; i<(m-1);i++){    
        for (int j=0; j<n_matches[i];j++){
            if (mask[i].at<uchar>(j,0) != 0)
                circle(img_matches[i], points[i][j], 10, Scalar(0,0,255), 3);
        }
    }

    //-- Show good matches
    //namedWindow("Good Matches", CV_WINDOW_NORMAL);
    //imshow("Good Matches", img_matches[0]);
    //waitKey();


    /* Triangulation */
    Mat points4D;   // homogeneous point world coordinates
    //@brief Reconstructs points by triangulation.
    //Keep in mind that all input data should be of float type in order for this function to work.

    triangulate_Points(R,t,points,pointsCompare,points4D);

    Mat points3D(3,n_matches[0],CV_32F);

    for (int i=0; i<n_matches[0]; i++)
    {
    	float x = points4D.at<float>(3,i);
        points3D.at<float>(0,i) = points4D.at<float>(0,i) / x;
        points3D.at<float>(1,i) = points4D.at<float>(1,i) / x;
        points3D.at<float>(2,i) = points4D.at<float>(2,i) / x;
   }

    PnP(good_matches,2,keypoints,R,t,points,pointsCompare);
    std::cout<<points3D.cols<<std::endl;
    add_Points(R,t,points,pointsCompare,points3D,2);
    std::cout<<points3D.cols<<std::endl;

    /* ----------------------- Visualization with RViz --------------------------*/
    invertpose(R,t,_R,_t);
    /* transform rotation matrix to quaternion */
    r2q(_R,q0,q);
    //cout << "Quaternion 1: "<< endl << "w: "<<q0.w()<<endl<<"vector: "<< endl<<q0.vec() << endl;
   // cout << "Quaternion 2: "<< endl << "w: "<<q[0].w()<<endl<<"vector: "<< endl<<q[0].vec() << endl;
    while (ros::ok()){
        viz(points3D,pub_pts,pub_cam,_t,mask,q,3);
    }

    delete n_matches;
    return 0;
}


//  cmd example: rosrun fyp_3d_reconstruction fyp_3d_reconstruction
