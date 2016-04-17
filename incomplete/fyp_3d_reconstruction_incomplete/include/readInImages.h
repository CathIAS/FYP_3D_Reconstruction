#ifndef READINIMAGES_H
#define READINIMAGES_H

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <dirent.h>  // for reading files from path
#include <sys/stat.h>
#include <sstream>  // string stream
#include <algorithm>  // sorting algorithm

#include "global.h"

/* Read in images from folder path */
void readInImages(cv::Mat img[], const std::string &folder, int &m);

/* Read in images from folder path */
void readFilenames(std::vector<std::string> &filenames, const std::string &directory);

/* Load images with filenames */
int loadImages(cv::Mat img[], std::vector<std::string> &filenames, const std::string &folder, int &m);

#endif /* READINIMAGES_H */

