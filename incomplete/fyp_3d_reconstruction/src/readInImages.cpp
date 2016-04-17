#include "readInImages.h"

using namespace std;
using namespace cv;

/* Read in images from folder path */

void readInImages(Mat img[], const string &folder, int &m)
{

	/* Obtain list of image file names */
    vector<string> filenames;  // a vector of the found names in the path
    readFilenames(filenames, folder);  // get names and assign them to vector

    sort(filenames.begin(), filenames.end());  // sort filenames

    cout << "images discovered under folder: " << endl;
    for (int i=0; i<filenames.size(); i++)
        cout << "   " << filenames[i] << endl;
    
    /* load images */
    int suc = loadImages(img, filenames, folder, m); // 0 - success, -1 - fail

    assert((suc==0)&& (m > 0)&&"No image was found! Aborting...");
    cout << "Success in loading images! Number of images loaded: " << m << endl;
}
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


/* Load images with filenames */
int loadImages(Mat img[], vector<string> &filenames, const string &folder, int &m)
{
    m = filenames.size(); // number of images
    for(size_t i=0; i<m; i++)
    {
        /* Load image */
        img[i] = imread(folder + "/" + filenames[i]);
        if(!img[i].data) 
        {
            cerr << "Problem loading image!" << endl;
            return -1;
        }
    }
    return 0;
}

