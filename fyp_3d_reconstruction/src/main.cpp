#include "ros/ros.h"

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
//#include "opencv2/sfm.hpp"
#include "opencv2/core/eigen.hpp"


#include <iostream>

#include "global.h"
#include "readInImages.h"
#include "surf.h"

using namespace std; 
using namespace cv;
using namespace Eigen;

/***********************************************/

/*                    Main                     */

/***********************************************/

int main(int argc, char** argv)
{
    Mat img[n];  // original images
    Mat img_undist[n];  // undistorted images

    vector< DMatch > good_matches;  // matches for drawing
    vector< Point2f > points_1, points_2;  // coordinates of matched points
    Mat img_matches;  // img with matches for display

    Mat E,R,t; // essential matrix, rotation matrix, cam translation
    Mat mask; // mask for inliers after RANSAC

    int m;  // number of images loaded


    /* --------------------- Specifies executable usage --------------------- */
    if(argc != 3)
    {
        cout << "Usage: ./fyp_3d_reconstruction <mode> [directory]" << endl;
        cout << "       <mode> : images" << endl;
        cout << "       [directory] : path to image folder. Do NOT add '/' at the end. Folder should only contain image files" << endl;
        return -1;
    }

    /* ------------ In "images" mode - reading images from folder ----------- */
    if (strcmp(argv[1], "images") == 0)  // if argv[1] is "images"
    {
        cout << "Mode selected: images" << endl;

        /* Get folder path from command line */ 
        const string folder = argv[2]; 
        cout << "Folder path: " << argv[2] << endl;

        /* Read in images from folder path */
        int check = readInImages(img, folder, m);

        /* Check if images are loaded successfully */
        if ((check == 0) && (m > 0))
            cout << "Success in loading images! Number of images loaded: " << m << endl;
        else if ((check == 0) && (m == 0)) {
            cout << "No image was found! Aborting..." << endl; 
            return -1;
        }
        else 
            return -1;
    }   

    /* ------------------------- 3D Reconstruction -------------------------- */
    /* Show Image size */
    cout << "Picture Height: " << img[0].rows << endl << "Picture Width: " << img[0].cols << endl;

    /* Undistortion with cam parameters */
    for (int i=0; i<m; i++)
        undistort(img[i], img_undist[i], camIntrinsic, dist);

    /* SURF detector for features and matching*/
    surf(img_undist, good_matches, points_1, points_2, 400, 0, 1, img_matches);

    //-- Show good matches
//    namedWindow("Good Matches", CV_WINDOW_NORMAL);
//    imshow("Good Matches", img_matches);
    
    cout<<"Total "<<points_1.size()<<" "<<"points have been found"<<endl;
//    for(int i=0;i<points_1.size();i++)
//        cout<<"x = "<<points_1[i]<<"	"<<"y = "<<points_2[i]<<endl;
//    waitKey();

    /* find essential matrix using five-point algorithm with RANSAC */
    E = findEssentialMat(points_1, points_2, camIntrinsic, RANSAC, 0.999, 1.0, mask);
    cout << "Essential Matrix: " << endl << " " << E << endl;

    /* use cheirality check to obtain R, t */
    recoverPose(E, points_1, points_2, camIntrinsic, R, t, mask);
    R.convertTo(R,CV_32FC1);
    t.convertTo(t,CV_32FC1);
    cout << "Rotation Matrix: " << endl << " " << R << endl;
    cout << "Translation: " << endl << " " << t << endl;

    /* transform rotation matrix to quaternion */
    Matrix3f R_eigen; // matrix representation in eigen library
    cv2eigen(R, R_eigen);
    Quaternionf q(R_eigen);
    q.normalize();
    cout << "quaternion: " << "w: "<<q.w()<< endl <<"vector: "<< endl<<q.vec() << endl;

    /* calculate projection matrices */
    Mat projMatr1(3,4,CV_32F);  // projection matrix
    Mat projMatr2(3,4,CV_32F);
    Mat points4D(4,points_1.size(),CV_32F);  // homogeneous point world coordinates

    Mat Rt(3,4,CV_32F);  // Rt = [R | t]
    Mat Rt_init = (Mat_<float>(3,4) << 1,0,0,0,0,1,0,0,0,0,1,0) ; // identity Rt

    hconcat(R, t, Rt);  // Rt concatenate
    projMatr1 = camIntrinsic * Rt_init;  // ProjMat = K * Rt
    projMatr2 = camIntrinsic * Rt;
    cout << "Rt_1: " << endl << " " << Rt_init << endl;
    cout << "Rt_2: " << endl << " " << Rt << endl;
    cout << "Proj_1: " << endl << " " << projMatr1 << endl;
    cout << "Proj_2: " << endl << " " << projMatr2 << endl;

    vector< Mat_<float> > listOfproj;  // vector of projection matrices
    listOfproj.push_back(projMatr1);
    listOfproj.push_back(projMatr2);

    /* Triangulation */
    triangulatePoints(listOfproj[0], listOfproj[1], points_1, points_2, points4D);
    cout << "Triangulation results: " << endl;
    for (int i=0; i<points_1.size(); i++)
        cout << " " << points4D.at<float>(0,i) << "," <<points4D.at<float>(1,i) << "," << points4D.at<float>(2,i) << "," << points4D.at<float>(3,i) << endl;

    return 0;
}


//  cmd example: ./fyp_3d_reconstruction images ~/photos
