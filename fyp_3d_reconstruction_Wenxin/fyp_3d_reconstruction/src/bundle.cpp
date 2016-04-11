#include "bundle.h"

using namespace std;
using namespace cv;
using namespace Eigen;


/* convert pixel frame to image frame at unit distance */

Point2f pix2img(Point2f pt_pix)
{
    Point2f pt_img;

    float fx = camIntrinsic.at<float>(0,0);
    float fy = camIntrinsic.at<float>(1,1);
    float cx = camIntrinsic.at<float>(0,2);
    float cy = camIntrinsic.at<float>(1,2);

    if (pt_pix.x >= 0)
    {
        pt_img.x = (pt_pix.x - cx) / fx;
        pt_img.y = (pt_pix.y - cy) / fy;
    }

    return pt_img;
}


/* Bundle Adjustment */

void bundle(cv::Mat R[], cv::Mat T[], std::vector<Point3f> pts3, std::vector< std::vector<cv::Point2f> > z, int m_cam, int n_pts)
{

    /* Input confirmation */

    cout << "----------------------------" << endl;
    cout << "Entering Bundle Adjustment!!" << endl;
    cout << "number of cameras: " << m_cam << endl;
    cout << "number of points: " << n_pts << endl << endl;

    cout << "3D points: " << endl;
    for (int i=0;i<n_pts;i++)
        cout << pts3[i].x << "," << pts3[i].y << "," << pts3[i].z << endl;
    cout << endl;

    cout << "2D points observation (pixel frame) on 1st cam: " << endl;
    for (int i=0;i<n_pts;i++)
        cout << z[0][i].x << "," << z[0][i].y << endl; 
    cout << endl;


    /* Converting to Eigen */

    Matrix3f Rot[m_cam];
    Vector3f Tra[m_cam];
    Vector3f P3[n_pts];

    for (int i=0;i<m_cam;i++)
    {
        cv2eigen(R[i],Rot[i]);
        cv2eigen(T[i],Tra[i]);
    }
    for (int j=0;j<n_pts;j++)
    {
        P3[j](0) = pts3[j].x;
        P3[j](1) = pts3[j].y;
        P3[j](2) = pts3[j].z;
    }


    /* Calculate Rotation Vector */

    Mat theta[m_cam];
    /* convert R to theta */
    for (int i=0; i<m_cam; i++)
        cv::Rodrigues(R[i], theta[i]);

    cout << "Rotation Vector: " << endl;
    cout << "theta[0]: " << theta[0] << endl;
    cout << "theta[1]: " << theta[1] << endl;
    cout << endl;


    /* Calculate reprojection f(x) given state vector x */

    vector< vector<Point2f> > rep;  // vector of reprojection points (mxn)
    rep.resize(m_cam);
    
    for (int i=0;i<m_cam;i++){
        for (int j=0;j<n_pts;j++){
    
            Vector3f P3_c;
            P3_c = Rot[i] * P3[j] + Tra[i];

            Point2f p;
            p.x = P3_c(0) / P3_c(2);
            p.y = P3_c(1) / P3_c(2);
            rep[i].push_back(p);
        }
    }

    cout << "Reprojection calculated!!!" << endl << endl;


    /* Visual comparison check between observationn and reprojection */
    
    vector< vector<Point2f> > z_img;  // vector of observation in image frame
    z_img.resize(m_cam);
    
    for (int i=0;i<m_cam;i++){
        for (int j=0;j<n_pts;j++){
            Point2f p;
            p = pix2img(z[i][j]);
            z_img[i].push_back(p);
        }
    }

    cout << "Observation (image frame) on 1st cam: " << endl;
    for (int i=0;i<n_pts;i++)
        cout << z_img[0][i].x << "," << z_img[0][i].y << endl; 
    cout << endl;

    cout << "Reprojection (image frame) on 1st cam: " << endl;
    for (int i=0;i<n_pts;i++)
        cout << rep[0][i].x << "," << rep[0][i].y << endl; 
    cout << endl;


    /* Define and calculate error vector */
    
    int comb = 2 * m_cam * n_pts;  // number of terms in an error vector
    VectorXf err(comb);  // error vector init

    for (int k=0;k<m_cam;k++){   // kth camera
        for (int j=0;j<n_pts;j++){   // jth point
            float ex = 0;
            float ey = 0;
            if (z[k][j].x >= 0){   // if jth pt can be seen from kth cam
                ex = rep[k][j].x - z_img[k][j].x;  // err in x
                ey = rep[k][j].y - z_img[k][j].y;  // err in y
            }
            int index = k*n_pts + j;
            err(2*index) = ex;
            err(2*index + 1) = ey;
        }
    }

    cout << "size of error vector: " << err.size() << endl;
    cout << "error vector value: " << err << endl << endl;


    /* calculation of Jacobian cam units 2x6 */
    
    MatrixXf J_c(2,6);


    /* calculation of Jacobian point units 2x3 */


    /* complete Jacobian matrix of error function */ 

}

