#include "surf.h"
#include "global.h"
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
//bool cmpMatch(DMatch d1,DMatch d2){ return d1.};

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
    for( int i = 0; i < descriptors_1.rows; i++ )
    { if( matches[i].distance <= max(3*min_dist, 0.03) )
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

void surf(const Mat img_undist[],vector< DMatch > good_matches[],vector< Point2f > points[],vector< Point2f > pointsCompare[],const int hessian,Mat img_matches[])
{
	/*variables*/
    vector<KeyPoint> keypoints, keypointsCompare[m-1];
    Mat gimg, gimgCompare[m-1];
	Mat descriptors, descriptorsCompare[m-1];
    std::vector< DMatch > matches[m-1];
    FlannBasedMatcher matcher;

	Ptr<SURF> detector = SURF::create( hessian );

		//-- Step 1: Detect the keypoints of mid-img using SURF Detector
		cv::cvtColor(img_undist[mid], gimg, cv::COLOR_BGR2GRAY);
		if( !gimg.data ){std::cout<< " --(!) Error reading images " << std::endl;}
		detector->detect( gimg, keypoints);
	    detector->compute( gimg, keypoints, descriptors );

	    //-- Step 2: Detect the keypoints of other imgs using SURF Detector, then match with mid-img
	    for(int i=0;i< m;i++){
	    	if(i == mid){continue;}
	    	else if(i<mid){

	    		cv::cvtColor(img_undist[i], gimgCompare[i], cv::COLOR_BGR2GRAY);
	    		if( !gimgCompare[i].data ){std::cout<< " --(!) Error reading images " << std::endl;}
	    		detector->detect( gimgCompare[i], keypointsCompare[i]);
	    		detector->compute( gimgCompare[i], keypointsCompare[i], descriptorsCompare[i] );
	    		matcher.match( descriptors, descriptorsCompare[i], matches[i]);
	    	}
	    	else if(i>mid){

	    		cv::cvtColor(img_undist[i], gimgCompare[i-1], cv::COLOR_BGR2GRAY);
	    		if( !gimgCompare[i-1].data ){std::cout<< " --(!) Error reading images " << std::endl;}
	    		detector->detect( gimgCompare[i-1], keypointsCompare[i-1]);
	    		detector->compute( gimgCompare[i-1], keypointsCompare[i-1], descriptorsCompare[i-1] );
	    		matcher.match( descriptors, descriptorsCompare[i-1], matches[i-1]);
	    	}
	    }

	    //-- Step 3: Calculate dist and select good matches for each pair

	    for(int i=0;i<(m-1);i++){
	        double max_dist = 0;
	        double min_dist = 100;
		//-- Quick calculation of max and min distances between keypoints for pair[i]
	    	for( int j = 0; j < descriptors.rows; j++){
	    		double dist = matches[i][j].distance;
	    		if( dist < min_dist ) min_dist = dist;
	    		if( dist > max_dist ) max_dist = dist;
	    	}

	    	printf("-- Max dist : %f \n", max_dist );
	    	printf("-- Min dist : %f \n", min_dist );

	    //only "good" matches
	    	for( int j = 0; j < descriptors.rows; j++ ){

	    		if( matches[i][j].distance <= max(3*min_dist, 0.03) ){
	    			good_matches[i].push_back( matches[i][j]);
	    		}
	    	}

        //sort the matches from mindis to maxdis for pair[i]
    		sort(good_matches[i].begin(),good_matches[i].end());

    	//convert the matches into points coordinates for pair[i]
    			for(int j = 0; j < good_matches[i].size(); j++)
    			{
    				points[i].push_back(keypoints[good_matches[i][j].queryIdx].pt);
    				pointsCompare[i].push_back(keypointsCompare[i][good_matches[i][j].trainIdx].pt);
    			}



	    /* number of feature points */
	    n_matches[i] = points[i].size();
        cout<<"Total "<< n_matches[i] <<" "<<"points have been found between mid-image"<<" and image"<<i<<endl;
    }

    /*  //-- Draw matches
        drawMatches( gimg_1, keypoints_1, gimg_2, keypoints_2, matches, img_matches );
        */


    Mat mask;
    Scalar color = Scalar::all(-1);
    int i=0;
	drawMatches( gimg, keypoints, gimgCompare[i], keypointsCompare[i], good_matches[i], img_matches[i], color, color, mask, 2);

}

