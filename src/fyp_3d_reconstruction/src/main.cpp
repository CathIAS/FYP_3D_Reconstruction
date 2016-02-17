#include "ros/ros.h"
#include "opencv2/opencv.hpp"

#include <iostream>
#include <dirent.h>  // for reading files from path
#include <sys/stat.h>
#include <sstream>  // string stream
#include <algorithm>  // include sorting algorithm

using namespace cv;
using namespace std;

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

int main(int argc, char** argv)
{
    cout << "Ready!" << endl;
//    cout << "Number of arguments: " << argc << endl;
//    cout << "argv[1] (mode): " << argv[1] << endl;

    /* Specifies executable usage */

    if(argc != 3)
    {
        cout << "Usage: ./fyp_3d_reconstruction <mode> [directory]" << endl;
        cout << "       <mode> : images" << endl;
        cout << "       [directory] : path to image folder. Do NOT add '/' at the end." << endl;
        return -1;
    }

    if (strcmp(argv[1], "images") == 0)  // if argv[1] is "images"
    {
        cout << "Entering mode: images" << endl;

        /* Read in images from folder path */
        
        const string folder = argv[2];  // get folder path from command line
        cout << "Folder path: " << argv[2] << endl;

        vector<string> filenames;  // initialize vector of filenames

        readFilenames(filenames, folder); // get filenames from folder
        sort(filenames.begin(), filenames.end()); // sort filenames
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
            
            /* Turn int to string */
            stringstream ss;
            ss << i+1;
            /* Show image */
            namedWindow("Image"+ss.str(), WINDOW_NORMAL);
            imshow("Image"+ss.str(),img[i]);
            waitKey();
            }

    }



    return 0;
}


//  Program executable is at <catkin_ws>/devel/lib/fyp_3d_reconstruction
