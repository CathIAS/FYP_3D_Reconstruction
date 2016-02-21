#include "ros/ros.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

#include "global.h"
#include "readInImages.h"

using namespace cv;
using namespace std;

/***********************************************/

/*                    Main                     */

/***********************************************/

int main(int argc, char** argv)
{
    Mat img[n];  // stores original images
    Mat img_dist[n];  // stores undistorted images

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
            undistort(img[i], img_dist[i], camIntrinsic, dist);

        /* Show image */
        for (int i=0; i<m; i++)
        {
            stringstream ss;  // turn int to string
            ss << i+1;
            namedWindow("Undistorted Image"+ss.str(), WINDOW_NORMAL);  // adjust display window size
            imshow("Undistorted Image"+ss.str(),img_dist[i]);  // show image in the window. "Image1"
            waitKey();
        }
        
        /* TODO SURF detector for features */

        /* THINK: how to arrange the code to do matching, as the match needs to be done 2 by 2 in order. (k-1, k) > (k, k+1) > ... It might be best to write SURF and matching as individual functions that can be called in main. */

    }

    return 0;
}


//  Program executable is at <catkin_ws>/devel/lib/fyp_3d_reconstruction
//  cmd example: ./fyp_3d_reconstruction images ~/photos
