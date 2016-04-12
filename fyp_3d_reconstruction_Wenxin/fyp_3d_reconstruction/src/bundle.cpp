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


/* Calculate projection Jacobian matrix*/
/*
 * J = [ 1/Z  0  -X/(Z^2) 
 *        0  1/Z -Y/(Z^2) ]
 */

MatrixXf cal_Jp(Matrix3f R, Vector3f T, Vector3f X)
{
    MatrixXf J(2,3);

    Vector3f v = R * X + T;
    J(0,0) = 1 / v(2);
    J(0,1) = 0;
    J(0,2) = -v(0) / (v(2)*v(2));
    J(1,0) = 0;
    J(1,1) = 1 / v(2);
    J(1,2) = -v(1) / (v(2)*v(2));

    return J;
}


/* Calculate error Jacobian of cam translation */
/*
 * Jt = [ 1/Z  0  -X/(Z^2) 
 *        0   1/Z -Y/(Z^2) ]
 */

MatrixXf cal_Jt(Matrix3f R, Vector3f T, Vector3f X)
{
    MatrixXf J(2,3);

    MatrixXf Jp(2,3);
    Jp = cal_Jp(R, T, X);

    J = Jp;

    return J;
}


/* Calculate error Jacobian of 3D point coordinates */
/*
 * Jx = [ 1/Z  0  -X/(Z^2)    *  R
 *        0   1/Z -Y/(Z^2) ]
 */

MatrixXf cal_Jx(Matrix3f R, Vector3f T, Vector3f X)
{
    MatrixXf J(2,3);

    MatrixXf Jp(2,3);
    Jp = cal_Jp(R, T, X);

    J = Jp * R;

    return J;
}


/* Calculate error Jacobian of cam rotation */
/*
 * Jr = [ 1/Z  0  -X/(Z^2)    *  R  *  [X x]^T
 *        0   1/Z -Y/(Z^2) ]
 */

MatrixXf cal_Jr(Matrix3f R, Vector3f T, Vector3f X)
{
    MatrixXf J(2,3);

    MatrixXf Jp(2,3);
    Jp = cal_Jp(R, T, X);

    Matrix3f X_cross;
    X_cross(0,0) = 0;
    X_cross(0,1) = X(2);
    X_cross(0,2) = -X(1);

    X_cross(1,0) = -X(2);
    X_cross(1,1) = 0;
    X_cross(1,2) = X(0);

    X_cross(2,0) = X(1);
    X_cross(2,1) = -X(0);
    X_cross(2,2) = 0;

    J = Jp * R * X_cross; 

    return J;
}



/* Bundle Adjustment */

void bundle(cv::Mat R[], cv::Mat T[], std::vector<Point3f> pts3, std::vector< std::vector<cv::Point2f> > z, int m_cam, int n_pts)
{

    /* Input confirmation */

    cout << "----------------------------" << endl;
    cout << "Entering Bundle Adjustment!!" << endl;
    cout << "number of cameras: " << m_cam << endl;
    cout << "number of points: " << n_pts << endl << endl;

    //    cout << "3D points: " << endl;
    //    for (int i=0;i<n_pts;i++)
    //        cout << pts3[i].x << "," << pts3[i].y << "," << pts3[i].z << endl;
    //    cout << endl;

    //    cout << "2D points observation (pixel frame) on 1st cam: " << endl;
    //    for (int i=0;i<n_pts;i++)
    //        cout << z[0][i].x << "," << z[0][i].y << endl; 
    //    cout << endl;


    /* Calculate Rotation Vector */

    Mat theta[m_cam];
    /* convert R to theta */
    for (int i=0; i<m_cam; i++)
        cv::Rodrigues(R[i], theta[i]);

    //    cout << "Rotation Vector: " << endl;
    //    cout << "theta[0]: " << theta[0] << endl;
    //    cout << "theta[1]: " << theta[1] << endl;
    //    cout << endl;


    /* Converting to Eigen */

    Matrix3f Rot[m_cam];
    Vector3f Theta[m_cam];
    Vector3f Tra[m_cam];
    Vector3f P3[n_pts];

    for (int i=0;i<m_cam;i++)
    {
        cv2eigen(R[i],Rot[i]);
        cv2eigen(T[i],Tra[i]);
        cv2eigen(theta[i],Theta[i]);
    }
    for (int j=0;j<n_pts;j++)
    {
        P3[j](0) = pts3[j].x;
        P3[j](1) = pts3[j].y;
        P3[j](2) = pts3[j].z;
    }


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

    //    cout << "Reprojection calculated!!!" << endl << endl;


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

    //    cout << "Observation (image frame) on 1st cam: " << endl;
    //    for (int i=0;i<n_pts;i++)
    //        cout << z_img[0][i].x << "," << z_img[0][i].y << endl; 
    //    cout << endl;

    //    cout << "Reprojection (image frame) on 1st cam: " << endl;
    //    for (int i=0;i<n_pts;i++)
    //        cout << rep[0][i].x << "," << rep[0][i].y << endl; 
    //    cout << endl;


    /* Define and calculate error vector */

    int n_row = 2 * m_cam * n_pts;  // number of terms in an error vector
    VectorXf err(n_row);  // error vector init

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

    //    cout << "size of error vector: " << err.size() << endl;
    //    cout << "error vector value: " << err << endl << endl;


    /* calculation of Jacobian units */

    /*
    // testing function:

    MatrixXf J_t(2,3);   // cam translation Jacobian - T
    MatrixXf J_r(2,3);   // cam rotation Jacobian - theta
    MatrixXf J_x(2,3);   // point Jacobian - X

    //    cout << "Rot[0]: " << Rot[0] << endl;
    //    cout << "Tra[0]: " << Tra[0] << endl;
    //    cout << "Theta[0]: " << Theta[0] << endl;
    //    cout << "P3[0]: " << P3[0] << endl << endl;

    //    cout << "Rot[1]: " << Rot[1] << endl;
    //    cout << "Tra[1]: " << Tra[1] << endl;
    //    cout << "Theta[1]: " << Theta[1] << endl;
    //    cout << "P3[1]: " << P3[1] << endl << endl;

    //    J_t = cal_Jt(Rot[0],Tra[0],P3[0]);
    //    J_r = cal_Jr(Rot[0],Tra[0],P3[0]);
    //    J_x = cal_Jx(Rot[0],Tra[0],P3[0]);

    J_t = cal_Jt(Rot[1],Tra[1],P3[1]);
    J_r = cal_Jr(Rot[1],Tra[1],P3[1]);
    J_x = cal_Jx(Rot[1],Tra[1],P3[1]);

    cout << "J_t: " << J_t << endl;
    cout << "J_r: " << J_r << endl;
    cout << "J_x: " << J_x << endl;
    */


    /* complete Jacobian matrix of error function */ 

    int n_col = 6*m_cam + 3*n_pts;
    MatrixXf J = MatrixXf::Zero(n_row, n_col);
    cout << "size of J: " << J.rows() << "x" << J.cols() << endl;

    for (int k=0;k<m_cam;k++){      // for each cam
        for (int j=0;j<n_pts;j++){      // for each pt
            if (z[k][j].x >= 0){   // if jth pt can be seen from kth cam

                int row_index = 2*(k*n_pts + j);
                int col_index_r = 6*k;
                int col_index_t = 6*k + 3;
                int col_index_x = 6*m_cam + 3*j;
//                cout << "row_index: " << row_index << endl;
//                cout << "col_index_r: " << col_index_r << endl;
//                cout << "col_index_t: " << col_index_t << endl;
//                cout << "col_index_x: " << col_index_x << endl;

                J.block(row_index,col_index_r,2,3) = cal_Jr(Rot[k],Tra[k],P3[j]);
                J.block(row_index,col_index_t,2,3) = cal_Jt(Rot[k],Tra[k],P3[j]);
                J.block(row_index,col_index_x,2,3) = cal_Jx(Rot[k],Tra[k],P3[j]);

            }
        }
    }

/*
    // check Jacobian structure - to cv image
    Mat J_cv, J_cv_binary;
    eigen2cv(J, J_cv);
    threshold(J_cv, J_cv_binary, 0, 255, THRESH_BINARY);
    namedWindow( "Jacobian", WINDOW_AUTOSIZE );
    imshow( "Jacobian", J_cv_binary );                
    waitKey();
*/


    /* Obtain Hessian Matrix and b vector for QR */
    
    MatrixXf H(n_col,n_col);
    VectorXf b(n_col);

    H = J.transpose() * J;
    b = J.transpose() * err;

    cout << "size of H: " << H.rows() << "x" << H.cols() << endl;
    cout << "size of b: " << b.rows() << "x" << b.cols() << endl;
   
/*
    // check Hessian structure - to cv image
    Mat H_cv, H_cv_binary;
    eigen2cv(H, H_cv);
    threshold(H_cv, H_cv_binary, 0, 255, THRESH_BINARY);
    namedWindow( "Hessian", WINDOW_AUTOSIZE );
    imshow( "Hessian", H_cv_binary );                
    waitKey();
*/


    /* QR to solve H * delta x = -b */

//    H.block(0,0,3,3) += 


}

