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
    const string folder = "/home/liuwx/git/FYP_3D_Reconstruction/photos/new_recons";
    cout << "Folder path: " << folder << endl;

    /* Read in images from folder path */
    readInImages(img, folder, m);
    /* Show Image size */
    cout << "Picture Height: " << img[0].rows << endl << "Picture Width: " << img[0].cols << endl;
    //    }   


    /* Variables */
    vector< DMatch > good_matches[m-1];  				// matches for drawing
    vector< Point2f > points[m-1], pointsCompare[m-1];  // matched points
    Mat img_matches[m-1];  								// img with matches for display
    Mat E[m-1];  										// essential matrix
    Mat mask[m-1]; 										// mask for inliers after RANSAC
    n_matches = new int[m];                             // number of matches for each pair
    mid = m/2;

    Mat R[m-1], t[m-1]; 								// R, t from the perspective cams
    Mat R_in1[m-1], t_in1[m-1];                         // R, t from the 1st cam frame
    Quaterniond q[m-1]; 								// quaternions for visualization


    /* ------------------------- 3D Reconstruction -------------------------- */

    /* Undistortion with cam parameters */
    undistort(img, img_undist, camIntrinsic, dist);

    /* SURF detector for features and matching*/
    surf(img_undist, good_matches, points, pointsCompare, 400, img_matches);

    /* find essential matrix using five-point algorithm with RANSAC */
    findEssentialMat(points, pointsCompare, camIntrinsic, RANSAC, 0.999, 1.0, mask, E);

    /* use cheirality check to obtain R, t */
    recoverPose(E, points, pointsCompare, camIntrinsic, R, t, mask);

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
    waitKey();

    Rt_invert(R, R_in1, t, t_in1);

    cout << "Rotation Matrix from perspective cam: " << endl << " " << R[0] << endl;
    cout << "Translation from perspective cam: " << endl << " " << t[0] << endl;
    cout << "Rotation Matrix from first cam: " << endl << " " << R_in1[0] << endl;
    cout << "Translation from first cam: " << endl << " " << t_in1[0] << endl;

    /* transform rotation matrix to quaternion */
    r2q(R_in1,q0,q);
    cout << "Quaternion 1: "<< endl << "w: "<<q0.w()<<endl<<"vector: "<< endl<<q0.vec() << endl;
    cout << "Quaternion 2: "<< endl << "w: "<<q[0].w()<<endl<<"vector: "<< endl<<q[0].vec() << endl;


    /* Triangulation */
    Mat points4D[m-1];  // homogeneous point world coordinates
// TODO when operating points4D things always go wrong here. 
    triangulatePoints(R,t,points,pointsCompare,points4D);

    // normalize homogeneous coordinates
    for (int i=0;i<(m-1);i++){
        for (int j=0;j<n_matches[i];j++){
            double x = points4D[i].at<double>(j,3);
            points4D[i].at<double>(j,0) = points4D[i].at<double>(j,0) / x;
            points4D[i].at<double>(j,1) = points4D[i].at<double>(j,1) / x;
            points4D[i].at<double>(j,2) = points4D[i].at<double>(j,2) / x;
            points4D[i].at<double>(j,3) = points4D[i].at<double>(j,3) / x;
        }
    }

    cout << "Triangulation results: " << endl;
    for (int i=0; i<n_matches[0]; i++)
        cout << " " << points4D[0].at<double>(i,0) << "," <<points4D[0].at<double>(i,1) << "," << points4D[0].at<double>(i,2) << ", " << points4D[0].at<double>(i,3) << endl;


    /* ----------------------- Visualization with RViz --------------------------*/

    while (ros::ok()){

    //        viz(points4D[3],pub_pts,pub_cam,t_in1,q); //TODO function poorly writteni

    }
    delete n_matches;
    return 0;

}

//  cmd example: rosrun fyp_3d_reconstruction fyp_3d_reconstruction
