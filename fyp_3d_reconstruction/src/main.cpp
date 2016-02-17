#include "ros/ros.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <dirent.h>  // for reading files from path
#include <sys/stat.h>
#include <sstream>  // string stream
#include <algorithm>  // sorting algorithm

using namespace cv;
using namespace std;


/***********************************************/

/*                 Functions                   */

/***********************************************/

/* TODO Set cam parameters */

/* Returns a list of files in a directory (except the ones that begin with a dot) */
void readFilenames(vector<string> &filenames, const string &directory)
{
    DIR *dir;
    class dirent *ent;
    class stat st;

    dir = opendir(directory.c_str());
    while ((ent = readdir(dir)) != NULL) {
        const string file_name = ent->d_name;
        const string full_file_name = directory + "/" + file_name;

        if (file_name[0] == '.')  // exclude files starting with .
            continue;

        if (stat(full_file_name.c_str(), &st) == -1)  // exclude nonexisting files
            continue;

        const bool is_directory = (st.st_mode & S_IFDIR) != 0;

        if (is_directory)
            continue;  // exclude directories

//        filenames.push_back(full_file_name); // returns full path
        filenames.push_back(file_name); // returns just filename
    }
    closedir(dir);
}  // Get files in directory


/***********************************************/

/*                    Main                     */

/***********************************************/

int main(int argc, char** argv)
{
    cout << "Ready!" << endl;

    /* Specifies executable usage */
    if(argc != 3)
    {
        cout << "Usage: ./fyp_3d_reconstruction <mode> [directory]" << endl;
        cout << "       <mode> : images" << endl;
        cout << "       [directory] : path to image folder. Do NOT add '/' at the end. Folder should only contain image files" << endl;
        return -1;
    }

    /* In "images" mode - reading images from folder */
    if (strcmp(argv[1], "images") == 0)  // if argv[1] is "images"
    {
        cout << "Entering mode: images" << endl;

        /* Read in images from folder path */
        const string folder = argv[2];  // get folder path from command line
        cout << "Folder path: " << argv[2] << endl;

        vector<string> filenames;  // a vector of the found names in the path

        readFilenames(filenames, folder);  // get names and assign them to vector
        sort(filenames.begin(), filenames.end());  // sort filenames
        cout << "images discovered under folder: " << endl;
        for (int i=0; i<filenames.size(); i++)
            cout << "   " << filenames[i] << endl;

        int n = filenames.size(); // number of images
        Mat img[n];

        for(size_t i=0; i<n; i++)
        {
            /* Load image */
            img[i] = imread(folder + "/" + filenames[i]);
            if(!img[i].data) 
            {
                cerr << "Problem loading image!" << endl;
                return -1;
            }
             
            /* Show image */
            stringstream ss;  // turn int to string
            ss << i+1;
            namedWindow("Image"+ss.str(), WINDOW_NORMAL);  // adjust display window size
            imshow("Image"+ss.str(),img[i]);  // show image in the window. "Image1"
            waitKey();
        }

        /* TODO Undistortion with cam parameters */
        /* TODO SURF detector for features */

        /* THINK: how to arrange the code to do matching, as the match needs to be done 2 by 2 in order. (k-1, k) > (k, k+1) > ... It might be best to write SURF and matching as individual functions that can be called in main. */

    }

    return 0;
}


//  Program executable is at <catkin_ws>/devel/lib/fyp_3d_reconstruction
//  cmd example: ./fyp_3d_reconstruction images ~/photos
