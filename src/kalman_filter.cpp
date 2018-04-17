#include <iostream>
#include "kalman_filter.h"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    /* predict the state */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    /* Update the state. Kalman filter update step. */
    VectorXd y_ = z - H_ * x_; // error calculation
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K =  P_ * H_.transpose() * S.inverse();
    
    // New state
    x_ = x_ + (K * y_);
    // New covariance
    int x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /* Update the state. Extended Kalman filter update step. */
   
    VectorXd h = VectorXd(3);
    
    const double THRESHOLD = 0.000001;
    
    const double px = x_(0);
    const double py = x_(1);
    const double vx = x_(2);
    const double vy = x_(3);
    
    const double rho = sqrt(px * px + py * py);
    const double theta = atan2(py, px);
    double rho_dot = 0.0;
    
    if (rho > THRESHOLD) {
        rho_dot = ( px * vx + py * vy ) / rho;
    } else {
        rho_dot = 0.0;
    }
    h << rho, theta, rho_dot;
    
    VectorXd y_ = z - h;
    y_(1) = atan2(sin(y_(1)), cos(y_(1)));  // Angle normalization
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K =  P_ * H_.transpose() * S.inverse();

    // New state
    x_ = x_ + (K * y_);
    // New covariance
    int x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}







