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
    const string folder = "/home/xsunaf/FYP/FYP_3D_Reconstruction/photos/test-2";
    cout << "Folder path: " << folder << endl;

    /* Read in images from folder path */
    readInImages(img, folder, m);
    /* Show Image size */
    cout << "Picture Height: " << img[0].rows << endl << "Picture Width: " << img[0].cols << endl;
    //    }   


    /* Variables */
    vector< DMatch > good_matches[m-1];  				// matches for drawing
    vector< Point2f > points[m-1], pointsCompare[m-1];  	// coordinates of matched points
    Mat img_matches[m-1];  								// img with matches for display
    Mat E[m-1];  										// essential matrix
    Mat mask[m-1]; 										// mask for inliers after RANSAC
    Mat R[m-1], t[m-1]; 								// rotation matrices, cam translations
    Quaterniond q[m-1]; 								// orientation quaternions
    n_matches = new int[m-1];
    mid = m/2;


    /* ------------------------- 3D Reconstruction -------------------------- */

    /* Undistortion with cam parameters */
    undistort(img, img_undist, camIntrinsic, dist);

    /* SURF detector for features and matching*/
    surf(img_undist, good_matches, points, pointsCompare, 400, img_matches);

    /* find essential matrix using five-point algorithm with RANSAC */
    findEssentialMat(points, pointsCompare, camIntrinsic, RANSAC, 0.999, 1.0, mask, E);

    /* use cheirality check to obtain R, t */
    recoverPose(E[0], points[0], pointsCompare[0], camIntrinsic, R[0], t[0], mask[0]);


    // Obtain inverse:
    Mat R_12[1], t_12[1];  // p_2 = R_12 * p_1 + t_12  representation of the first cam in the second
    Mat R_21[1], t_21[1];  // representation of the second cam in the first

    R_12[0] = R[0];
    t_12[0] = t[0];
    R_21[0] = R_12[0].inv();
    t_21[0] = -1 * R_21[0] * t_12[0];

    /* draw RANSAC inliers in red */
    for (int i=0; i<(m-1);i++){    
        for (int j=0; j<n_matches[i];j++){
            if (mask[i].at<uchar>(j,0) != 0)
                circle(img_matches[i], points[i][j], 10, Scalar(0,0,255), 3);
        }
    }

    //-- Show good matches
    namedWindow("Good Matches", CV_WINDOW_NORMAL);
    imshow("Good Matches", img_matches[0]);
    //waitKey();

    cout << "Rotation Matrix 1: " << endl << " " << R0 << endl;
    cout << "Translation 1: " << endl << " " << t0 << endl;
    cout << "Rotation Matrix 2: " << endl << " " << R[0] << endl;
    cout << "Translation 2: " << endl << " " << t[0] << endl;

    /* transform rotation matrix to quaternion */
    r2q(R_21,q0,q);
    cout << "Quaternion 1: "<< endl << "w: "<<q0.w()<<endl<<"vector: "<< endl<<q0.vec() << endl;
    cout << "Quaternion 2: "<< endl << "w: "<<q[0].w()<<endl<<"vector: "<< endl<<q[0].vec() << endl;


    /* Triangulation */
    Mat points4D[m-1];  // homogeneous point world coordinates
    //@brief Reconstructs points by triangulation.
    //Keep in mind that all input data should be of float type in order for this function to work.
    triangulatePoints(R,t,points,pointsCompare,points4D);

    Mat points3D(3,n_matches[0],CV_32F);
    for (int i=0; i<n_matches[0]; i++)
    {
    	float x = points4D[0].at<float>(3,i);
        points3D.at<float>(0,i) = points4D[0].at<float>(0,i) / x;
        points3D.at<float>(1,i) = points4D[0].at<float>(1,i) / x;
        points3D.at<float>(2,i) = points4D[0].at<float>(2,i) / x;
   }

    solvePnPRansac(points3D,points[2],camIntrinsic,dist,R[2],t[2]);


    /* ----------------------- Visualization with RViz --------------------------*/
    while (ros::ok()){
        viz(points3D,pub_pts,pub_cam,t_21,mask,q);
    }
    delete n_matches;
    return 0;
}


//  cmd example: rosrun fyp_3d_reconstruction fyp_3d_reconstruction
