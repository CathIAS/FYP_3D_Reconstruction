#include "surf.h"

//bool cmpMatch(DMatch d1,DMatch d2){ return d1.};

void surf(Mat img_undist[],vector< DMatch >& good_matches,vector< Point2f >& points_1,vector< Point2f >& points_2,const int hessian,const int idx_1,const int idx_2,Mat& img_matches)
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
    for( int i = 0; i < descriptors_1.rows; i++ )
    { if( matches[i].distance <= max(2*min_dist, 0.02) )
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

