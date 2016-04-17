// TODO: the includes between files are not elegant enough. Some header files are included more than once. If there is a better way to organize our header files and includes, please do it now.
//
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>


#include <iostream>

#include "global.h"
#include "readInImages.h"
#include "surf.h"

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

    // TODO: change pub_cam topic type to MarkerArray s.t. multiple cameras can be displayed. If possible, please use tf library to display multiple axes instead of using arrows in marker type.
    //
    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

    ros::Publisher pub_cam = nh.advertise<visualization_msgs::Marker>("recon_cam", 10);

    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

    /* Variables */

    Mat img[N];  // original images
    Mat img_undist[N];  // undistorted images

    int m;  // number of images loaded
    int n;  // number of matched points

    // TODO: these variables below are not compatible with multiple images input. Please change them into arrays and also change accordingly in the many functions below. Note that essential matrix, img_matches and R_eigen can be temporary variables, and might not need to be changed to arrays. Please decide at will from the sense of coding neatness and formality when making changes. 
    //
    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

    vector< DMatch > good_matches;  // matches for drawing
    vector< Point2f > points_1, points_2;  // position of matched points on Mat
    Mat img_matches;  // img with matches for display

    Mat E;  // essential matrix
    Mat mask; // mask for inliers after RANSAC
    Matrix3d R_eigen; // matrix representation in eigen library

    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$



    // TODO: obviously right now using rosnode, we cannot pass argv as before. Please read ros documentations and if necessary use "rosparam" functionality to achieve folder path input again.
    //
    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

    //    /* --------------------- Specifies executable usage --------------------- */
    //    if(argc != 3)
    //    {
    //        cout << "Usage: ./fyp_3d_reconstruction <mode> [directory]" << endl;
    //        cout << "       <mode> : images" << endl;
    //        cout << "       [directory] : path to image folder. Do NOT add '/' at the end. Folder should only contain image files" << endl;
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
    int check = readInImages(img, folder, m);

    /* Check if images are loaded successfully */
    if ((check == 0) && (m > 0))
        cout << "Success in loading images! Number of images loaded: " << m << endl;
    else if ((check == 0) && (m == 0)) {
        cout << "No image was found! Aborting..." << endl; 
        return -1; }
    else 
        return -1;
    //    }   

    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$


    /* ------------------------- 3D Reconstruction -------------------------- */

    Mat R[m], t[m]; // rotation matrices, cam translations
    Quaterniond q[m]; // orientation quaternions

    /* initialization of the first camera R,t,q */
    R[0] = (Mat_<float>(3,3) << 1,0,0,0,1,0,0,0,1);
    t[0] = (Mat_<float>(3,1) << 0,0,0);
    cv2eigen(R[0], R_eigen); // convert R from cv type to eigen type
    q[0] = R_eigen; // transform into quaternion


    /* Show Image size */
    cout << "Picture Height: " << img[0].rows << endl << "Picture Width: " << img[0].cols << endl;

    /* Undistortion with cam parameters */
    for (int i=0; i<m; i++)
        undistort(img[i], img_undist[i], camIntrinsic, dist);

    // TODO: from the first visualization result, we can see that only taking the "good" image matches is far from enough. The number of points is not enough to do a good reconstruction. Please find a way to increase the number of points that can be used for reconstruction, or, increase n. Note that RANSAC normally cannot take matches with more than 1/3 outliers.
    //
    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

    /* SURF detector for features and matching*/
    surf(img_undist, good_matches, points_1, points_2 ,400 ,4 , 5, img_matches);

    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$


    /* number of feature points */
    n = points_1.size(); 
    cout<<"Total "<< n <<" "<<"points have been found"<<endl;

    // circle all good points in first image
    for (int i=0;i<n;i++)
        circle(img_matches, points_1[i], 10, Scalar(0,255,0), 3);

    //-- Show good matches
    namedWindow("Good Matches", CV_WINDOW_NORMAL);
    imshow("Good Matches", img_matches);
    waitKey();


    /* find essential matrix using five-point algorithm with RANSAC */
    E = findEssentialMat(points_1, points_2, camIntrinsic, RANSAC, 0.999, 1.0, mask);
    //    cout << "Essential Matrix: " << endl << " " << E << endl;

    /* use cheirality check to obtain R, t */
    int inliers = recoverPose(E, points_1, points_2, camIntrinsic, R[1], t[1], mask);
    R[1].convertTo(R[1],CV_32F);
    t[1].convertTo(t[1],CV_32F);

    //    Mat R_1, R_2, t_1;
    //    decomposeEssentialMat(E, R_1, R_2, t_1);
    //    cout << "decompose essential matrix possible results: " << endl;
    //    cout << "R1: " << endl << " " << R_1 << endl;
    //    cout << "R2: " << endl << " " << R_2 << endl;
    //    cout << "t: " << endl << " " << t_1 << endl;

    // scaling of t
    t[1] =  0.552 * t[1]; // 0.552 meters

    // Obtain inverse:
    Mat R_12, t_12;  // p_2 = R_12 * p_1 + t_12  representation of the first cam in the second
    Mat R_21, t_21;  // representation of the second cam in the first

    R_12 = R[1];
    t_12 = t[1];
    R_21 = R_12.inv();
    t_21 = -1 * R_21 * t_12;

    cout << "Number of inliers: " << inliers << endl;
    cout << "Rotation Matrix R_11: " << endl << " " << R[0] << endl;
    cout << "Translation t_11: " << endl << " " << t[0] << endl;
    cout << "Rotation Matrix R_12: " << endl << " " << R[1] << endl;
    cout << "Translation t_12: " << endl << " " << t[1] << endl;
    cout << "Rotation Matrix R_21: " << endl << " " << R_21 << endl;
    cout << "Translation t_21: " << endl << " " << t_21 << endl;


    /* transform rotation matrix to quaternion */
    cv2eigen(R_21, R_eigen);
    q[1] = R_eigen;
    q[1].normalize();
    cout << "Quaternion 1: " << "w: "<<q[0].w()<<endl<<"vector: "<< endl<<q[0].vec() << endl;
    cout << "Quaternion 2: " << "w: "<<q[1].w()<<endl<<"vector: "<< endl<<q[1].vec() << endl;


    // TODO: please identify which variables are temporary variables and which are better to be declared at the beginning, and modify names such as "Rt" and "Rt_init", make them recognizable when multiple images are present.
    //
    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

    /* calculate projection matrices */
    Mat projMatr1(3,4,CV_32F);  // projection matrices
    Mat projMatr2(3,4,CV_32F);

    Mat points4D(4,n,CV_32F);  // homogeneous point world coordinates
    Mat points3D(3,n,CV_32F);

    Mat Rt(3,4,CV_32F);  // Rt = [R | t]
    Mat Rt_init = (Mat_<float>(3,4) << 1,0,0,0,0,1,0,0,0,0,1,0) ; // identity Rt

    hconcat(R[1], t[1], Rt);  // Rt concatenate
    projMatr1 = camIntrinsic * Rt_init;  // ProjMat = K * Rt
    projMatr2 = camIntrinsic * Rt;
    //    cout << "Rt_1: " << endl << " " << Rt_init << endl;
    //    cout << "Rt_2: " << endl << " " << Rt << endl;
    //    cout << "Proj_1: " << endl << " " << projMatr1 << endl;
    //    cout << "Proj_2: " << endl << " " << projMatr2 << endl;

    vector< Mat_<float> > listOfproj;  // vector of projection matrices
    listOfproj.push_back(projMatr1);
    listOfproj.push_back(projMatr2);

    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

    //    cout << "mask: " << endl << " " << mask << endl;

    /* Triangulation */
    triangulatePoints(listOfproj[0], listOfproj[1], points_1, points_2, points4D);
    //    cout << "Triangulation results: " << endl;
    for (int i=0; i<n; i++)
    {
        //        cout << " " << points4D.at<float>(0,i) << "," <<points4D.at<float>(1,i) << "," << points4D.at<float>(2,i) << "," << points4D.at<float>(3,i) << endl;

        float x = points4D.at<float>(3,i); 

        points3D.at<float>(0,i) = points4D.at<float>(0,i) / x;
        points3D.at<float>(1,i) = points4D.at<float>(1,i) / x;
        points3D.at<float>(2,i) = points4D.at<float>(2,i) / x;

        //        cout << " " << points3D.at<float>(0,i) << "," <<points3D.at<float>(1,i) << "," << points3D.at<float>(2,i) << endl;
    }
    // homogenious to normal 3D:



    /* ----------------------- Visualization with RViz --------------------------*/
    visualization_msgs::Marker pts_array, cam_1, cam_2;

    pts_array.header.frame_id = "/world_frame";
    cam_1.header.frame_id = "/world_frame";
    cam_2.header.frame_id = "/world_frame";

    pts_array.header.stamp = ros::Time::now();
    cam_1.header.stamp = ros::Time::now();
    cam_2.header.stamp = ros::Time::now();

    pts_array.ns = "points_and_cameras";
    cam_1.ns = "points_and_cameras";
    cam_2.ns = "points_and_cameras";
    pts_array.action = visualization_msgs::Marker::ADD;
    cam_1.action = visualization_msgs::Marker::ADD;
    cam_2.action = visualization_msgs::Marker::ADD;

    pts_array.id = 0;
    cam_1.id = 1;
    cam_2.id = 2;

    pts_array.type = visualization_msgs::Marker::POINTS;
    cam_1.type = visualization_msgs::Marker::ARROW;
    cam_2.type = visualization_msgs::Marker::ARROW;

    pts_array.scale.x = 0.01;
    pts_array.scale.y = 0.01;
    pts_array.scale.z = 0.01;

    cam_1.scale.x = 0.1;
    cam_1.scale.y = 0.1;
    cam_1.scale.z = 0.1;
    cam_2.scale.x = 0.1;
    cam_2.scale.y = 0.1;
    cam_2.scale.z = 0.1;

    // TODO: Here all the points colors are in red. If possible change them according to their pixel value in the photos.
    //
    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

    pts_array.color.a = 1.0;
    pts_array.color.r = 1.0;
    pts_array.color.g = 0.0;
    pts_array.color.b = 0.0;

    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

    cam_1.color.a = 1.0;
    cam_1.color.r = 0.0;
    cam_1.color.g = 0.0;
    cam_1.color.b = 1.0;
    cam_2.color.a = 1.0;
    cam_2.color.r = 0.0;
    cam_2.color.g = 1.0;
    cam_2.color.b = 0.0;

    /* cameras are initialized with Markers::pose */

    cam_1.pose.position.x = t[0].at<float>(0);
    cam_1.pose.position.y = t[0].at<float>(1);
    cam_1.pose.position.z = t[0].at<float>(2);
    cam_2.pose.position.x = t_21.at<float>(0);
    cam_2.pose.position.y = t_21.at<float>(1);
    cam_2.pose.position.z = t_21.at<float>(2);

    cam_1.pose.orientation.x = q[0].x();
    cam_1.pose.orientation.y = q[0].y();
    cam_1.pose.orientation.z = q[0].z();
    cam_1.pose.orientation.w = q[0].w();
    cam_2.pose.orientation.x = q[1].x();
    cam_2.pose.orientation.y = q[1].y();
    cam_2.pose.orientation.z = q[1].z();
    cam_2.pose.orientation.w = q[1].w();

    /* points are initialized with geometry_msgs::Point */

    int inlier_count = 0;
    int outlier_count = 0;
    for (int i=0;i<n;i++)
    {
        if (mask.at<uchar>(i,0) != 0)
        {         
            geometry_msgs::Point p;
            p.x = points3D.at<float>(0,i);
            p.y = points3D.at<float>(1,i);
            p.z = points3D.at<float>(2,i);

            pts_array.points.push_back(p);
            inlier_count++;
        }
//        else
//        {
//            outlier_count++;
//            cout << "outlier number " << outlier_count << endl;
//        }
    }

    while (ros::ok())
    {

        pub_pts.publish(pts_array);
        pub_cam.publish(cam_1);
        pub_cam.publish(cam_2);

    }
    return 0;
}


//  cmd example: rosrun fyp_3d_reconstruction fyp_3d_reconstruction
