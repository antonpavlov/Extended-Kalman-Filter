#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

#define EPS 0.0001 // A very small number
#define EPS2 0.0000001

//Tools::Tools() {}

//Tools::~Tools() {}

//VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
//                              const vector<VectorXd> &ground_truth) {
//    /*Calculate the RMSE */
//
//        VectorXd rmse(4);
//        rmse << 0,0,0,0;
//
//        // check the validity of the following inputs:
//        //  * the estimation vector size should not be zero
//        //  * the estimation vector size should equal ground truth vector size
//        if(estimations.size() != ground_truth.size() || estimations.size() == 0){
//            cout << "Invalid estimation or ground_truth data" << endl;
//            return rmse;
//        }
//
//        //accumulate squared residuals
//        for(unsigned int i=0; i < estimations.size(); ++i){
//
//            VectorXd residual = estimations[i] - ground_truth[i];
//
//            //coefficient-wise multiplication
//            residual = residual.array()*residual.array();
//            rmse += residual;
//        }
//
//        //calculate the mean
//        rmse = rmse/estimations.size();
//
//        //calculate the squared root
//        rmse = rmse.array().sqrt();
//
//        //return the result
//        return rmse;
//}
//
//MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
//    /* Calculate a Jacobian */
//
//    MatrixXd Hj_(3, 4);
//
//    Hj_ << (x_state[0]/sqrt(pow(x_state[0],2)+pow(x_state[1],2))), (x_state[1]/sqrt(pow(x_state[0],2)+pow(x_state[1],2))), 0, 0,
//    -(x_state[1]/(pow(x_state[0],2)+pow(x_state[1],2))), (x_state[0]/(pow(x_state[0],2)+pow(x_state[1],2))), 0, 0,
//    ((x_state[1]*((x_state[2]*x_state[1])-(x_state[3]*x_state[0])))/(cbrt(pow((pow(x_state[0],2))+(pow(x_state[1],2)),2)))),
//    ((x_state[0]*((x_state[3]*x_state[0])-(x_state[2]*x_state[1])))/(cbrt(pow((pow(x_state[0],2))+(pow(x_state[1],2)),2)))),
//    (x_state[0]/sqrt(pow(x_state[0],2)+pow(x_state[1],2))), (x_state[1]/sqrt(pow(x_state[0],2)+pow(x_state[1],2)));
//
//    return Hj_;
//}


VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    // Check the validity of the following inputs:
    // The estimation vector size should not be zero
    if(estimations.size() == 0){
        cout << "Input is empty" << endl;
        return rmse;
    }
    // The estimation vector size should equal ground truth vector size
    if(estimations.size() != ground_truth.size()){
        cout << "Invalid estimation or ground_truth. Data should have the same size" << endl;
        return rmse;
    }
    // Accumulate squared residuals
    for(unsigned int i=0; i < estimations.size(); ++i){
        VectorXd residual = estimations[i] - ground_truth[i];
        // Coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }
    
    // Calculate the mean
    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
//    // Code from lectures quizes
//    float px = x_state(0);
//    float py = x_state(1);
//    float vx = x_state(2);
//    float vy = x_state(3);
//    MatrixXd Hj(3,4);
//
////    // Deal with the special case problems
////    if (fabs(px) < EPS){
////        if (px > 0){
////            px = EPS;
////        } else {
////            px = -EPS;
////        }
////    }
////
////    if (fabs(py) < EPS){
////         if (py > 0){
////             py = EPS;
////         } else {
////             py = -EPS;
////         }
////
////    }
////    // Pre-compute a set of terms to avoid repeated calculation
////    float c1 = px*px+py*py;
////    // Check division by zero
////    if(fabs(c1) < EPS2){
////        c1 = EPS2;
////    }
////    float c2 = sqrt(c1);
////    float c3 = (c1*c2);
////    // Compute the Jacobian matrix
////    Hj <<   (px/c2),    (py/c2),                            0,      0,
////            -(py/c1),   (px/c1),                            0,      0,
////            py*(vx*py - vy*px)/c3,  px*(px*vy - py*vx)/c3,  px/c2,  py/c2;
//
////    if (px == 0) {
////        cout <<"Division by zero detected!"<< endl;
////        px = 0.0001;
////    } else if (py == 0){
////        cout <<"Division by zero detected!"<< endl;
////        py = 0.0001;
////    }
//
//
//    double var_0_0 = (px/sqrt(pow(px,2)+pow(py,2)));
//    double var_0_1 = (py/sqrt(pow(px,2)+pow(py,2)));
//    double var_0_2 = 0;
//    double var_0_3 = 0;
//
//    double var_1_0 = -(py/(pow(px,2)+pow(py,2)));
//    double var_1_1 = (px/(pow(px,2)+pow(py,2)));
//    double var_1_2 = 0;
//    double var_1_3 = 0;
//
//    double var_2_0 = (py * ( (vx*py)-(vy*px) ) ) /    ( cbrt(pow(  pow(px,2) + pow(py,2),2) ) );
//    double var_2_1 = (px * ( (vy*px) - (vx*py) ))/    ( cbrt(pow(  pow(px,2) + pow(py,2),2) ) );
//    double var_2_2 = (px/sqrt(pow(px,2)+pow(py,2)));
//    double var_2_3 = (py/sqrt(pow(px,2)+pow(py,2)));
//
////    Hj <<   (px/sqrt(pow(px,2)+pow(py,2))),     (py/sqrt(pow(px,2)+pow(py,2))), 0, 0,
////    -(py/(pow(px,2)+pow(py,2))),        (px/(pow(px,2)+pow(py,2))), 0, 0,
////    ((py*((vx*py)-(vy*px)))/(cbrt(pow((pow(px,2))+(pow(py,2)),2)))),
////    ((px*((vy*px)-(vx*py)))/(cbrt(pow((pow(px,2))+(pow(py,2)),2)))),
////    (px/sqrt(pow(px,2)+pow(py,2))), (py/sqrt(pow(px,2)+pow(py,2)));
//
//    Hj <<   var_0_0,    var_0_1,    var_0_2,    var_0_3,
//            var_1_0,    var_1_1,    var_1_2,    var_1_3,
//            var_2_0,    var_2_1,    var_2_2,    var_2_3;
//
//    return Hj;
    
    
    const double THRESH = 0.0001;
    MatrixXd H = MatrixXd::Zero(3, 4);
    
    const double px = x_state(0);
    const double py = x_state(1);
    const double vx = x_state(2);
    const double vy = x_state(3);
    
    const double d_squared = px * px + py * py;
    const double d = sqrt(d_squared);
    const double d_cubed = d_squared * d;
    
    if (d >= THRESH){
        
        const double r11 = px / d;
        const double r12 = py / d;
        const double r21 = -py / d_squared;
        const double r22 = px / d_squared;
        const double r31 = py * (vx * py - vy * px) / d_cubed;
        const double r32 = px * (vy * px - vx * py) / d_cubed;
        
        H << r11, r12, 0.0, 0.0,
        r21, r22, 0.0, 0.0,
        r31, r31, r11, r12;
    }
    
    return H;
    
}
