#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <dirent.h>  // for reading files from path
#include <sys/stat.h>
#include <sstream>  // string stream
#include <algorithm>  // sorting algorithm

#include "global.h"

using namespace cv;
using namespace std;

/* Read in images from folder path */
int readInImages(Mat img[], const string &folder, int &m);

/* Read in images from folder path */
void readFilenames(vector<string> &filenames, const string &directory);

/* Load images with filenames */
int loadImages(Mat img[], vector<string> &filenames, const string &folder, int &m);

