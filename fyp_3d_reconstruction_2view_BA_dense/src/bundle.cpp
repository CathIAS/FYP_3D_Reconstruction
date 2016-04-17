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

MatrixXf cal_Jp(const Matrix3f R, const Vector3f T, const Vector3f X)
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

MatrixXf cal_Jt(const Matrix3f R, const Vector3f T, const Vector3f X)
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

MatrixXf cal_Jx(const Matrix3f R, const Vector3f T, const Vector3f X)
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

MatrixXf cal_Jr( const Matrix3f R, const Vector3f T, const Vector3f X)
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


void toEigen( const Mat R[], const Mat T[], const Mat theta[], const vector<Point3f> pts3, Matrix3f Rot[], Vector3f Theta[], Vector3f Tra[], Vector3f P3[], int m, int n)
{    
    for (int i=0;i<m;i++)
    {
        cv2eigen(R[i],Rot[i]);
        cv2eigen(T[i],Tra[i]);
        cv2eigen(theta[i],Theta[i]);
    }
    for (int j=0;j<n;j++)
    {
        P3[j](0) = pts3[j].x;
        P3[j](1) = pts3[j].y;
        P3[j](2) = pts3[j].z;
    }
}


void toCV( Mat R[], Mat T[], Mat theta[], vector<Point3f>& pts3, const Matrix3f Rot[], const Vector3f Theta[], const Vector3f Tra[], const Vector3f P3[], int m, int n)
{    
    for (int i=0;i<m;i++)
    {
        eigen2cv(Rot[i],R[i]);
        eigen2cv(Tra[i],T[i]);
        eigen2cv(Theta[i],theta[i]);
    }
    for (int j=0;j<n;j++)
    {
        pts3[j].x = P3[j](0);
        pts3[j].y = P3[j](1);
        pts3[j].z = P3[j](2);
    }
}


vector< vector<Point2f> >  getReprojection( const Vector3f Theta[], const Vector3f Tra[], const Vector3f P3[], int m, int n)
{
    vector< vector<Point2f> > rep;  // vector of reprojection points (mxn)
    rep.resize(m);

    for (int i=0;i<m;i++){
        for (int j=0;j<n;j++){

            Mat theta, R;
            Matrix3f Rot;
            eigen2cv(Theta[i], theta);
            Rodrigues(theta, R);
            cv2eigen(R, Rot);

            Vector3f P3_c;
            P3_c = Rot * P3[j] + Tra[i];

            Point2f p;
            p.x = P3_c(0) / P3_c(2);
            p.y = P3_c(1) / P3_c(2);
            rep[i].push_back(p);
            
//            if(i==1 && j==1)
//                cout << "selected reprojection: " << p.x<<","<< p.y<<endl;
        }
    }

    return rep;
}


vector< vector<Point2f> >  getObservation( const std::vector< std::vector<cv::Point2f> >& z, int m, int n)
{
    vector< vector<Point2f> > z_img;  // vector of reprojection points (mxn)
    z_img.resize(m);

    for (int i=0;i<m;i++){
        for (int j=0;j<n;j++){
            Point2f p;
            p = pix2img(z[i][j]);
            z_img[i].push_back(p);
            
//            if(i==1 && j==1)
//                cout <<setprecision(10) <<  "selected observation: " << p.x<<","<< p.y<<endl;
        }
    }

    return z_img;
}


void updateError( const std::vector< std::vector<cv::Point2f> >& z, const vector< vector<Point2f> >& rep, const vector< vector<Point2f> >& z_img, const int m_cam, const int n_pts, VectorXf& err)
{

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
/*            
            if(k==1 && j==1)
            {
                cout << "-----------------------" << endl;
                cout << "z.x: " << z[k][j].x << endl; 
                cout << "x: rep - z_img " << rep[k][j].x << " - " << z_img[k][j].x << endl; 
                cout << "y: rep - z_img " << rep[k][j].y << " - " << z_img[k][j].y << endl; 
                cout << "selected err: " << ex<<","<< ey<<endl;
                cout << "-----------------------" << endl;
            }
*/
        }
    }

}


void getJacobian( const std::vector< std::vector<cv::Point2f> >& z, const Matrix3f Rot[], const Vector3f Tra[], const Vector3f P3[], int m_cam, int n_pts, MatrixXf& J)
{
    for (int k=0;k<m_cam;k++){      // for each cam
        for (int j=0;j<n_pts;j++){      // for each pt
            if (z[k][j].x >= 0){   // if jth pt can be seen from kth cam

                int row_index = 2*(k*n_pts + j);
                int col_index_r = 6*k;
                int col_index_t = 6*k + 3;
                int col_index_x = 6*m_cam + 3*j;

                J.block(row_index,col_index_r,2,3) = cal_Jr(Rot[k],Tra[k],P3[j]);
                J.block(row_index,col_index_t,2,3) = cal_Jt(Rot[k],Tra[k],P3[j]);
                J.block(row_index,col_index_x,2,3) = cal_Jx(Rot[k],Tra[k],P3[j]);

            }
        }
    }

}

void getJnH( const std::vector< std::vector<cv::Point2f> >& z, const Matrix3f Rot[], const Vector3f Tra[], const Vector3f P3[], int m_cam, int n_pts, MatrixXf& J, MatrixXf& H)
{
    for (int k=0;k<m_cam;k++){      // for each cam
        for (int j=0;j<n_pts;j++){      // for each pt
            if (z[k][j].x >= 0){   // if jth pt can be seen from kth cam

                int row_index = 2*(k*n_pts + j);
                int col_index_r = 6*k;
                int col_index_t = 6*k + 3;
                int col_index_x = 6*m_cam + 3*j;

                MatrixXf Jr(2,3);   // cam rotation Jacobian - theta
                MatrixXf Jt(2,3);   // cam translation Jacobian - T
                MatrixXf Jx(2,3);   // point Jacobian - X
                MatrixXf Jc(2,6);
                
                Jr = cal_Jr(Rot[k],Tra[k],P3[j]);
                Jt = cal_Jt(Rot[k],Tra[k],P3[j]);
                Jx = cal_Jx(Rot[k],Tra[k],P3[j]); 
                Jc << Jr, Jt;

                J.block(row_index,col_index_r,2,3) = Jr;
                J.block(row_index,col_index_t,2,3) = Jt;
                J.block(row_index,col_index_x,2,3) = Jx;

                MatrixXf Jx_t = Jx.transpose();  // 3x2
                MatrixXf Jc_t = Jc.transpose();  // 6x2

                H.block(col_index_r,col_index_r,6,6) += Jc_t * Jc;
                H.block(col_index_x,col_index_x,3,3) += Jx_t * Jx;
                H.block(col_index_x,col_index_r,3,6) += Jx_t * Jc;
                H.block(col_index_r,col_index_x,6,3) += Jc_t * Jx;

            }
        }
    }



}
    
void increX( const VectorXf& del_x, Vector3f Theta[], Matrix3f Rot[], Vector3f Tra[], Vector3f P3[], int m_cam, int n_pts)
{
    for (int k=0;k<m_cam;k++)
    {
        Theta[k](0) += del_x(6*k);
        Theta[k](1) += del_x(6*k + 1);
        Theta[k](2) += del_x(6*k + 2);

        Tra[k](0) += del_x(6*k+3);
        Tra[k](1) += del_x(6*k+4);
        Tra[k](2) += del_x(6*k+5);

        Mat theta, R;
        eigen2cv(Theta[k], theta);
        Rodrigues(theta, R);
        cv2eigen(R, Rot[k]);
    }

    for (int j=0;j<n_pts;j++)
    {
        P3[j](0) += del_x(6*m_cam + 3*j); 
        P3[j](1) += del_x(6*m_cam + 3*j + 1); 
        P3[j](2) += del_x(6*m_cam + 3*j + 2); 
    }
}



/* =================================================================================== */

/* -------------------------------- Bundle Adjustment -------------------------------- */

/* =================================================================================== */

void bundle(cv::Mat R[], cv::Mat T[], std::vector<Point3f>& pts3, const std::vector< std::vector<cv::Point2f> >& z, int m_cam, int n_pts)
{
    
    /* ---------------------------- Input confirmation ---------------------------- */
    
    cout << endl;
    cout << "=========================================" << endl;
    cout << "Entering Bundle Adjustment!!" << endl;
    cout << "number of cameras: " << m_cam << endl;
    cout << "number of points: " << n_pts << endl;

    /*
       cout << "3D points: " << endl;
       for (int i=0;i<n_pts;i++)
       cout << pts3[i].x << "," << pts3[i].y << "," << pts3[i].z << endl;
       cout << endl;

       cout << "2D points observation (pixel frame) on 1st cam: " << endl;
       for (int i=0;i<n_pts;i++)
       cout << z[0][i].x << "," << z[0][i].y << endl; 
       cout << endl;
       */


    /* ------------------------ Variables declaration and init -------------------- */

    cout << "----------------------------" << endl;
    cout << "initializing variables" << endl;

    Mat theta[m_cam];   // rotation vector
    /* get theta from R */
    for (int i=0; i<m_cam; i++)
        cv::Rodrigues(R[i], theta[i]);

    Matrix3f Rot[m_cam];   // Eigen rotation matrix
    Vector3f Theta[m_cam];   // Eigen rotation vector
    Vector3f Tra[m_cam];   // Eigen translation vector
    Vector3f P3[n_pts];   // Eigen 3D point vector
    /* get Eigen from Mat */
    toEigen(R, T, theta, pts3, Rot, Theta, Tra, P3, m_cam, n_pts);

    vector< vector<Point2f> > rep;  // vector of reprojection points (mxn)
    vector< vector<Point2f> > z_img;  // vector of observation in image frame
    /* obtain observation vectors */
    z_img = getObservation(z, m_cam, n_pts);

    int n_row = 2 * m_cam * n_pts;  // size of error vector
    int n_col = 6*m_cam + 3*n_pts;  // size of state vector
    
    VectorXf err(n_row);  // error vector init 
    float err_sum;  // sum of square error
    float ini_err_sum;  // initial error sum
    float err_ded;  // error deduction rate

    MatrixXf J = MatrixXf::Zero(n_row, n_col);  // Jacobian matrix init
    MatrixXf H = MatrixXf::Zero(n_col, n_col);   // Hessian Matrix
    VectorXf b(n_col);    // vector b
    VectorXf del_x(n_col);   // delta x 

    cout << "variable initialization complete" << endl;
    cout << "size of J: " << J.rows() << "x" << J.cols() << endl;
    cout << "size of H: " << H.rows() << "x" << H.cols() << endl;
    cout << "size of b: " << b.rows() << "x" << b.cols() << endl;
    
    /* get initial reprojection */
    rep = getReprojection(Theta, Tra, P3, m_cam, n_pts);
    /* calculate initial error */
    updateError(z, rep, z_img, m_cam, n_pts, err);

    ini_err_sum = err.dot(err);
    cout << endl << "sum of square errors (initial): " << ini_err_sum << endl;

    /* ---------------------------------------------------------------------------- */

    int it = 3;
    cout << "----------------------------" << endl;
    cout << "Entering iterations" << endl;
    cout << "Number of iterations to go through: " << it << endl << endl;
/*   
//    cout << "Rot[0]: " << Rot[0] << endl;
    cout << "Tra[0]: " << Tra[0] << endl;
    cout << "Theta[0]: " << Theta[0] << endl;
    cout << "P3[0]: " << P3[0] << endl << endl;

//    cout << "Rot[1]: " << Rot[1] << endl;
    cout << "Tra[1]: " << Tra[1] << endl;
    cout << "Theta[1]: " << Theta[1] << endl;
    cout << "P3[1]: " << P3[1] << endl << endl;
    */

    int loop = 0;
    while (loop < it)
    {
   /* 
       cout << "Observation (image frame) on 1st cam: " << endl;
       for (int i=0;i<n_pts;i++)
       cout << z_img[0][i].x << "," << z_img[0][i].y << endl; 
       cout << endl;

       cout << "Reprojection (image frame) on 1st cam: " << endl;
       for (int i=0;i<n_pts;i++)
       cout << rep[0][i].x << "," << rep[0][i].y << endl; 
       cout << endl;
    */

//    cout << "size of error vector: " << err.size() << endl;
      
/*    
    // testing Jacobian:

    MatrixXf J_t(2,3);   // cam translation Jacobian - T
    MatrixXf J_r(2,3);   // cam rotation Jacobian - theta
    MatrixXf J_x(2,3);   // point Jacobian - X

//    cout << "Rot[0]: " << Rot[0] << endl;
//    cout << "Tra[0]: " << Tra[0] << endl;
//    cout << "Theta[0]: " << Theta[0] << endl;
//    cout << "P3[0]: " << P3[0] << endl << endl;

    cout << "Rot[1]: " << Rot[1] << endl;
    cout << "Tra[1]: " << Tra[1] << endl;
    cout << "Theta[1]: " << Theta[1] << endl;
    cout << "P3[1]: " << P3[1] << endl << endl;

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

    // numerical diff test
/*
    // theta:
    float diff = 1e-6;
    Vector3f Diff(0, 0, diff);
    Vector3f Tra_inc[m_cam];
    for (int i=0;i<m_cam;i++)
        Tra_inc[i] = Tra[i] + Diff;
    rep = getReprojection(Theta, Tra_inc, P3, m_cam, n_pts);
    updateError(z, rep, z_img, m_cam, n_pts, err);
*/
    
    
    /* complete Jacobian matrix of error function */ 
//    getJacobian(z, Rot, Tra, P3, m_cam, n_pts, J);
    /* Obtain Hessian Matrix and b vector for QR */
//    H = J.transpose() * J;

    getJnH(z, Rot, Tra, P3, m_cam, n_pts, J, H);

    b = J.transpose() * err;
   
    /* 
    // check Jacobian structure - to cv image
    Mat J_cv, J_cv_binary;
    eigen2cv(J, J_cv);
    threshold(J_cv, J_cv_binary, 0, 255, THRESH_BINARY);
    namedWindow( "Jacobian", WINDOW_AUTOSIZE );
    imshow( "Jacobian", J_cv_binary );                
    waitKey();
    */

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

//    FullPivLU<MatrixXf> lu_decomp(H);
//    cout << "determinant of H: " << H.determinant() << endl;
//    cout << "rank of H is: " << lu_decomp.rank() << endl;
//    FullPivLU<MatrixXf> lu_decomp_p(H);
//    cout << "rank of H after adding I: " << lu_decomp_p.rank() << endl;
//    cout << "determinant of H after adding I: " << H.determinant() << endl;

//    MatrixXf Id = MatrixXf::Identity(6,6);
//    H.block(0,0,6,6) = H.block(0,0,6,6) + Id;  // fix the first camera
    
    MatrixXf Id = MatrixXf:: Identity(n_col, n_col);
    float lamda = 5e-6;
    H += lamda * Id;

    float begin = clock();

    LLT<MatrixXf> dec(H);

    del_x = dec.solve(-b);
    cout << "***Time for solving linear system: " << float(clock() - begin)/CLOCKS_PER_SEC <<" sec" << endl;

//    cout << "rank of H (from ColQR): " << dec.rank() << endl;
//    cout << "determinant of H: " << H.determinant() << endl;

//    cout << "size of del_x: " << del_x.rows() << "x" << del_x.cols() << endl;
//    cout << "delta x solution is: " << endl << del_x << endl;

    increX(del_x, Theta, Rot, Tra, P3, m_cam, n_pts);

    rep = getReprojection(Theta, Tra, P3, m_cam, n_pts);
    updateError(z, rep, z_img, m_cam, n_pts, err);

    err_sum = err.dot(err);
    cout << "sum of square errors after loop " << loop+1 << ": " << err_sum << endl << endl;
 
    loop++;
    }

/*
//    cout << "Rot[0]: " << Rot[0] << endl;
    cout << "Tra[0]: " << Tra[0] << endl;
    cout << "Theta[0]: " << Theta[0] << endl;
    cout << "P3[0]: " << P3[0] << endl << endl;

//    cout << "Rot[1]: " << Rot[1] << endl;
    cout << "Tra[1]: " << Tra[1] << endl;
    cout << "Theta[1]: " << Theta[1] << endl;
    cout << "P3[1]: " << P3[1] << endl << endl;
*/

    err_ded = (ini_err_sum - err_sum) / ini_err_sum;

    cout << "----------------------------" << endl;
    cout << "Bundle Adjustment completed!!" << endl << endl;
    cout << "Percentage of error deducted in "<< it << " loops: " << err_ded << endl;
    cout << "=========================================" << endl;

    toCV(R, T, theta, pts3, Rot, Theta, Tra, P3, m_cam, n_pts);
    cout << "Updated state variables returned" << endl << endl;

}

