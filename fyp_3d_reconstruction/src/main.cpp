#include "ros/ros.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>

#include "global.h"
#include "readInImages.h"
#include "surf.h"

using namespace std;
using namespace cv;

/***********************************************/

/*                    Main                     */

/***********************************************/

int main(int argc, char** argv)
{
    Mat img[n];  // stores original images
    Mat img_undist[n];  // stores undistorted images
    vector< DMatch > good_matches;  //stores matches for drawing
	vector< Point2f > points_1, points_2;  //stores coordinates of match point
	Mat img_matches;  //img out with matches
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
        int m;  // number of images loaded
        int check = readInImages(img, folder, m);

        /* Check if images are loaded successfully */
        if (check == 0)
            cout << "Success in loading images! Number of images loaded: " << m << endl;
        else 
            return -1;

        /* Undistortion with cam parameters */
        for (int i=0; i<m; i++)
            undistort(img[i], img_undist[i], camIntrinsic, dist);

        /* Show image */
        for (int i=0; i<m; i++)
        {
            stringstream ss;  // turn int to string
            ss << i+1;
            namedWindow("Undistorted Image"+ss.str(), WINDOW_NORMAL);  // adjust display window size
            imshow("Undistorted Image"+ss.str(),img_undist[i]);  // show image in the window. "Image1"
            waitKey();
        }
        
        /* SURF detector for features and matching*/
        surf(img_undist,good_matches,points_1,points_2,400,7,8,img_matches);
        
				//-- Show good matches
				namedWindow("Good Matches", CV_WINDOW_NORMAL);
				imshow("Good Matches", img_matches);
				//print out coordinates
					cout<<"Total "<<points_1.size()<<" "<<"points have been found"<<endl;
				for(int i=0;i<points_1.size();i++){
					cout<<"x = "<<points_1[i]<<"	"<<"y = "<<points_2[i]<<endl;
				}
				waitKey();
        /* Input: Mat img_undist[n]; Output:vector<point2f> points1, points2. Arrays of matched points in two images correspondingly. Coordinates in floating points format. */
        /* THINK: how to arrange the code to do matching, as the match needs to be done 2 by 2 in order. (k-1, k) > (k, k+1) > ... It might be best to write SURF and matching as individual functions that can be called in main. */

        /* TODO find fundamental matrices and essential matrices */
        /* Input: vector<point2f> points1, points2; Output: Mat F & E */

    }

    return 0;
}


//  Program executable is at <catkin_ws>/devel/lib/fyp_3d_reconstruction
//  cmd example: ./fyp_3d_reconstruction images ~/photos
