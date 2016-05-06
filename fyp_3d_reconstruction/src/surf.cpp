#include "surf.h"
#include "global.h"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

//bool cmpMatch(DMatch d1,DMatch d2){ return d1.};


/******** Surf for two images **********/

void surf(const Mat img_undist[],vector< DMatch > good_matches,vector< Point2f > points_1,vector< Point2f > points_2,const int hessian,const int idx_1,const int idx_2,Mat img_matches)
{
    vector<KeyPoint> keypoints_1,keypoints_2;
    Mat gimg_1;
    Mat gimg_2;
    cv::cvtColor(img_undist[idx_1], gimg_1, cv::COLOR_BGR2GRAY);            
    cv::cvtColor(img_undist[idx_2], gimg_2, cv::COLOR_BGR2GRAY);            

    if( !gimg_1.data || !gimg_2.data )
    { std::cout<< " --(!) Error reading images " << std::endl;  }

    //-- Step 1: Detect the keypoints using SURF Detector

    Ptr<SURF> detector = SURF::create( hessian );
    detector->detect( gimg_1, keypoints_1);
    detector->detect( gimg_2, keypoints_2);


    Mat descriptors_1, descriptors_2;    
    detector->compute( gimg_1, keypoints_1, descriptors_1 );
    detector->compute( gimg_2, keypoints_2, descriptors_2 );  

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match( descriptors_1, descriptors_2, matches );

    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_1.rows; i++ )
    { double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );

    /*  //-- Draw matches
        Mat img_matches;
        drawMatches( gimg_1, keypoints_1, gimg_2, keypoints_2, matches, img_matches );
        */

    //only "good" matches
    for( int i = 0; i < descriptors_1.rows; i++ ){ 
        if( matches[i].distance <= max(3*min_dist, 0.03) )
        { good_matches.push_back( matches[i]); }
    }

    //sort the matches from mindis to maxdis
    sort(good_matches.begin(),good_matches.end());

    //convert the matches into points coordinates
    for(size_t i = 0; i < good_matches.size(); i++)
    {
        points_1.push_back(keypoints_1[good_matches[i].queryIdx].pt);
        points_2.push_back(keypoints_2[good_matches[i].trainIdx].pt);
    }

    Mat mask;
    Scalar color = Scalar::all(-1);
    drawMatches( gimg_1, keypoints_1, gimg_2, keypoints_2, good_matches, img_matches, color, color, mask, 2);

}


/********** Surf for multiple images **********/

void surf(const Mat img_undist[],vector< DMatch > good_matches[],std::vector<cv::KeyPoint> keypoints[],vector< Point2f > points[],vector< Point2f > pointsCompare[],const int hessian,Mat img_matches[])
{
    /*variables*/
    Mat gimg[m];  // grey images
    Mat descriptors[m];  // descriptors
    std::vector< DMatch > matches[m-1];
    FlannBasedMatcher matcher;

    Ptr<SURF> detector = SURF::create( hessian );

    //-- Step 1: Detect the keypoints of all imgs using SURF Detector
    for(int i=0;i< m;i++){
        cv::cvtColor(img_undist[i], gimg[i], cv::COLOR_BGR2GRAY);
        if( !gimg[i].data ){std::cout<< " --(!) Error reading images " << std::endl;}
        detector->detect( gimg[i], keypoints[i]);
        detector->compute( gimg[i], keypoints[i], descriptors[i] );
    }
    //-- Step 2: Then match with each img with its previous img
    //-- Step 3: find the good matches
    for(int i=0;i< (m-1);i++){
        matcher.match( descriptors[i], descriptors[i+1], matches[i]);
        findgoodmatches(matches[i],good_matches[i]);
        dmatch2p(good_matches[i],keypoints[i],keypoints[i+1],points[i],pointsCompare[i]);
        /* number of feature points */
        n_matches[i] = points[i].size();
        cout<<"Total "<< n_matches[i] <<" "<<"points have been found between image"<<i<<" and image"<<i+1<<endl;
    }




    Mat mask;
    Scalar color = Scalar::all(-1);
    // create img_matches for visualization
    for (int i=0;i<(m-1);i++){
        drawMatches( gimg[i], keypoints[i], gimg[i+1], keypoints[i+1], good_matches[i], img_matches[i], color, color, mask, 2);
        // circle the keypoints 
        for (int j=0;j<n_matches[i];j++)
            circle(img_matches[i], points[i][j], 10, Scalar(0,255,0), 3);
    }

}

void findgoodmatches(std::vector< DMatch >& match,std::vector< DMatch >& good_matches){
    double max_dist = 0;
    double min_dist = 100;
    //-- Quick calculation of max and min distances between keypoints for pair[i]
    for( int i = 0; i < match.size(); i++){
        double dist = match[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );

    //only "good" matches
    for( int i = 0; i < match.size(); i++ ){

        if( match[i].distance <= max(4*min_dist, 0.02) ){
            good_matches.push_back( match[i]);
        }
    }

    //sort the matches from min_dis to max_dis for pair[i]
    sort(good_matches.begin(),good_matches.end());
}

void dmatch2p(const std::vector< DMatch > match,const vector<KeyPoint> keypoints_q,const vector<KeyPoint> keypoints_t,vector< Point2f >& points,vector< Point2f >& pointsCompare){
    for(int i = 0; i < match.size(); i++){
        points.push_back(keypoints_q[match[i].queryIdx].pt);
        pointsCompare.push_back(keypoints_t[match[i].trainIdx].pt);
    }

}
