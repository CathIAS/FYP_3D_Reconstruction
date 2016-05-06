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


    /* Image variables */
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

//    /* ------------ In "images" mode - reading images from folder ----------- */
//    if (strcmp(argv[1], "images") == 0)  // if argv[1] is "images"
//        cout << "Mode selected: images" << endl;


    /* Get folder path */ 
    const string folder = "/home/liuwx/git/FYP_3D_Reconstruction/photos/test-4";
    //const string folder = "/home/xsunaf/FYP/FYP_3D_Reconstruction/photos/test-4";
    cout << "Folder path: " << folder << endl;

    /* Read in images from folder path */
    readInImages(img, folder, m);
    /* Show Image size */
    cout << "Picture Height: " << img[0].rows << endl << "Picture Width: " << img[0].cols << endl;


    /* Variables */
    Mat R[m],t[m]; 								    // rotation matrices, cam translations
    Quaterniond q[m]; 								// orientation quaternions
    
    vector<KeyPoint> keypoints[m];                  // SURF feature points
    vector< DMatch > good_matches[m-1];  			// matching index
    vector< Point2f > points[m-1], pointsCompare[m-1],mask3D[m];  	// coordinates of matched points, and mask3D showing relation between landmarks and cameras
    Mat img_matches[m-1];  							// img with matches for display
    n_matches = new int[m-1];
    
    Mat E;  										// essential matrix
    Mat mask; 										// mask for inliers after RANSAC

    R[0]=(Mat_<float>(3,3) << 1,0,0,0,1,0,0,0,1);
    t[0]=(Mat_<float>(3,1) << 0,0,0);


    /* ------------------------- 3D Reconstruction -------------------------- */

    /* Undistortion with cam parameters */
    undistort(img, img_undist, camIntrinsic, dist);

    /* SURF detector for features and matching*/
    surf(img_undist, good_matches,keypoints, points, pointsCompare, 4000, img_matches);

    /* find essential matrix using five-point algorithm with RANSAC */
    E= findEssentialMat(points[0], pointsCompare[0], camIntrinsic, RANSAC, 0.999, 1.0, mask);

    /* use cheirality check to obtain R, t */
    recoverPose(E, points[0], pointsCompare[0], camIntrinsic, R[1], t[1], mask);
    t[1] = 0.448 * t[1];

    // mask out wrong 2d points
    vector< Point2f > pointsx,pointsComparex;
    for(int i=0;i<points[0].size();i++){
    	if (mask.at<uchar>(i,0) != 0){
    		pointsx.push_back(points[0][i]);
    		pointsComparex.push_back(pointsCompare[0][i]);
    	}
    }
    std::cout<<"first 2 images mask result: "<<points[0].size()<<" -> "<<pointsx.size()<<std::endl;


    /* Triangulation */
    Mat points4D;   // homogeneous point world coordinates
    triangulate_init(R[1],t[1],pointsx,pointsComparex,points4D,mask3D);

    // 4d to 3d
    Mat points3D(3,points4D.cols,CV_32F);
    for (int i=0; i<points4D.cols; i++) {
    	float x = points4D.at<float>(3,i);
        points3D.at<float>(0,i) = points4D.at<float>(0,i) / x;
        points3D.at<float>(1,i) = points4D.at<float>(1,i) / x;
        points3D.at<float>(2,i) = points4D.at<float>(2,i) / x;
    }
    n = points3D.cols;

    // visualize two-view results
    viz(points3D,pub_pts,pub_cam,t,R,q,2);

    std::cout<<"number of 3D points by first 2 images: "<<n<<std::endl;
    std::cout<< "-------------------------------" << std::endl;


    /* multiple views and BA */
    for (int i=2; i<m-1;i++)
    {
        // get R and t of the newest cam
        PnP(good_matches,i,keypoints,R,t,points,pointsCompare,mask3D,img_matches[i-1]);
        std::cout<< "-------------------------------" << std::endl;
        // take deducted matches and triangulate
        add_Points(R,t,points,pointsCompare,points3D,i,mask3D,img_matches[i-1]);
        // visualize front-end
        viz(points3D,pub_pts,pub_cam,t,R,q,i+1);
        // BA
        bundle(R, t, points3D, mask3D, i+1, n, 4);
        // visualize back-end
        viz(points3D,pub_pts,pub_cam,t,R,q,i+1);
    }

    // get R and t of the newest cam
    PnP(good_matches,m-1,keypoints,R,t,points,pointsCompare,mask3D,img_matches[m-2]);
    std::cout<< "-------------------------------" << std::endl;
    // take deducted matches and triangulate
    add_Points(R,t,points,pointsCompare,points3D,m-1,mask3D,img_matches[m-2]);
    // visualize front-end
    viz(points3D,pub_pts,pub_cam,t,R,q,m);
    // BA
    bundle(R, t, points3D, mask3D, m, n, 8);
    
    // finalize visualization
    while (ros::ok()){
        viz(points3D,pub_pts,pub_cam,t,R,q,m);
    }

    
    /* ----------------------- Simulation ----------------------------*/
/*
    cout << "##################################" << endl;
    cout << "Entering Simulation" << endl;


    int m_cams = 3;
    int n_ptss = 18;
    Mat Rs[m_cams],ts[m_cams];
    Mat _Rs[m_cams],_ts[m_cams];
    Quaterniond qs[m_cams]; 								// orientation quaternions
    vector<Point3f> pts3Ds;
    Mat thetas[m_cams];   // rotation vector
    vector< vector<Point2f> > imgs;  // vector of reprojection points (mxn)
    vector< vector<Point2f> > pixs;  // vector of reprojection points (mxn)
    imgs.resize(m_cams);
    pixs.resize(m_cams);

    Rs[0] = R0;
    Rs[1] = R0;
    Rs[2] = R0;

    ts[0] = t0;
    ts[1] = (Mat_<float>(3,1) << -0.5,0,0);
    ts[2] = (Mat_<float>(3,1) << -1,0,0);
    
    Point3f p1(0.5,0,5);
    Point3f p2(0.5,0.2,5);
    Point3f p3(0.3,0.2,5);
    Point3f p4(0.3,0,5);
    Point3f p5(0.3,-0.2,5);
    Point3f p6(0.5,-0.2,5);
    Point3f p7(0.7,-0.2,5);
    Point3f p8(0.7,0,5);
    Point3f p9(0.7,0.2,5);
    Point3f p11(0.5,0,5.1);
    Point3f p12(0.5,0.2,5.1);
    Point3f p13(0.3,0.2,5.1);
    Point3f p14(0.3,0,5.1);
    Point3f p15(0.3,-0.2,5.1);
    Point3f p16(0.5,-0.2,5.1);
    Point3f p17(0.7,-0.2,5.1);
    Point3f p18(0.7,0,5.1);
    Point3f p19(0.7,0.2,5.1);
    pts3Ds.push_back(p1);
    pts3Ds.push_back(p2);
    pts3Ds.push_back(p3);
    pts3Ds.push_back(p4);
    pts3Ds.push_back(p5);
    pts3Ds.push_back(p6);
    pts3Ds.push_back(p7);
    pts3Ds.push_back(p8);
    pts3Ds.push_back(p9);
    pts3Ds.push_back(p11);
    pts3Ds.push_back(p12);
    pts3Ds.push_back(p13);
    pts3Ds.push_back(p14);
    pts3Ds.push_back(p15);
    pts3Ds.push_back(p16);
    pts3Ds.push_back(p17);
    pts3Ds.push_back(p18);
    pts3Ds.push_back(p19);

    // get theta from R 
    for (int i=0; i<m_cams; i++)
        cv::Rodrigues(Rs[i], thetas[i]);
    Matrix3f Rots[m_cams];   // Eigen rotation matrix
    Vector3f Thetas[m_cams];   // Eigen rotation vector
    Vector3f Tras[m_cams];   // Eigen translation vector
    Vector3f P3s[n_ptss];   // Eigen 3D point vector
    // get Eigen from Mat 
    toEigen(Rs, ts, thetas, pts3Ds, Rots, Thetas, Tras, P3s, m_cams, n_ptss);
    
    // get projection of the simulation points onto unit image plane 
    imgs = getReprojection(Thetas, Tras, P3s, m_cams, n_ptss);
    // get projection on pixel frame 
    for (int i=0;i<m_cams;i++){
        for (int j=0;j<n_ptss;j++){
            Point2f p;
            p = img2pix(imgs[i][j]);
            pixs[i].push_back(p);
        }
    }
    

//    cout << "2D points observation (pixel frame) on 1st cam: " << endl;
//    for (int i=0;i<n_ptss;i++)
//    cout << pixs[0][i].x << "," << pixs[0][i].y << endl;
//    cout << endl;
//    cout << "2D points observation (pixel frame) on 2st cam: " << endl;
//    for (int i=0;i<n_ptss;i++)
//    cout << pixs[1][i].x << "," << pixs[1][i].y << endl;
//    cout << endl;
//    cout << "2D points observation (pixel frame) on 3st cam: " << endl;
//   for (int i=0;i<n_ptss;i++)
//   cout << pixs[2][i].x << "," << pixs[2][i].y << endl;
//    cout << endl;


    vector< vector <Point2f> > vecs;  // rows - cams, cols - 3Dpoints; in pixel frame
    vecs.resize(m_cams);
    for (int i=0;i<n_pts;i++)
    {
            vecs[0].push_back(pixs[0][i]);
            vecs[1].push_back(pixs[1][i]);
            vecs[2].push_back(pixs[2][i]);
    }


    pts3Ds[0].x += 0.5;
    pts3Ds[1].y += 0.3;
    pts3Ds[2].z -= 0.2;
    pts3Ds[5].x += 0.6;
    pts3Ds[11].y -= 0.4;
    pts3Ds[13].x += 0.1;
    pts3Ds[17].x += 1;


//    for (int i=0;i<n_ptss;i++)
//        pts3Ds[i].x += 3;

    ts[2].at<float>(1) += 0.5;

    Mat theta_e = (Mat_<float>(3,1) << -0.56, 0.91, 0.13);
    Mat R_e;
    Rodrigues(theta_e, R_e);
    Rs[2] = Rs[2] * R_e;
    
    bundle(Rs, ts, pts3Ds, vecs, m_cams, n_ptss);
    
    while (ros::ok()){
        viz(points3D,pub_pts,pub_cam,t,R,q,m);
    }
*/ 

    delete n_matches;
    return 0;
}

