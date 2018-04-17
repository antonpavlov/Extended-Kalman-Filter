#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,  const vector<VectorXd> &ground_truth) {
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
    const double THRESHOLD = 0.0001; // Division by zero prevention
    MatrixXd Hj_ = MatrixXd::Zero(3, 4);
    
    const double px = x_state(0);
    const double py = x_state(1);
    const double vx = x_state(2);
    const double vy = x_state(3);
    
    const double divider_squared = px * px + py * py;
    const double divider = sqrt(divider_squared);
    const double divider_cubed = divider_squared * divider;
    
    if (divider >= THRESHOLD) {
        const double var_0_0 = px / divider;
        const double var_0_1 = py / divider;
        const double var_0_2 = 0;
        const double var_0_3 = 0;
        
        const double var_1_0 = -py / divider_squared;
        const double var_1_1 = px / divider_squared;
        const double var_1_2 = 0;
        const double var_1_3 = 0;
        
        const double var_2_0 = py * (vx * py - vy * px) / divider_cubed;
        const double var_2_1 = px * (vy * px - vx * py) / divider_cubed;
        const double var_2_2 = px / divider;
        const double var_2_3 = py / divider;  
        
        Hj_ <<  var_0_0,    var_0_1,    var_0_2,    var_0_3,
                var_1_0,    var_1_1,    var_1_2,    var_1_3,
                var_2_0,    var_2_1,    var_2_2,    var_2_3;
        
    }
    
    return Hj_;  // Return the Jacobian    
}
