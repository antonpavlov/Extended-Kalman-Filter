//#include <iostream>
//#include <vector>
//#include <iomanip>
//#include <stdexcept>
//#include "kalman_filter.h"
//
//using namespace Eigen;
//using namespace std;
//
//KalmanFilter::KalmanFilter(
//                           double dt,
//                           const Eigen::MatrixXd& F,
//                           const Eigen::MatrixXd& H,
//                           const Eigen::MatrixXd& Q,
//                           const Eigen::MatrixXd& R,
//                           const Eigen::MatrixXd& P,
//                           const Eigen::MatrixXd& candidateP)
//: F(F), H(H), Q(Q), R(R), P0(P), candidateP(P),
//m((int)H.rows()), n((int)F.rows()), dt(dt), initialized(false),
//I(n, n), estimatedState(n), estimatedCandidate(n)
//{
//    I.setIdentity();
//}
//
//KalmanFilter::KalmanFilter() {}
//
//void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
//    //Steup initial state vector with provided values
//    estimatedState = x0;
//    P = P0;
//    this->t0 = t0;
//    t = t0;
//    initialized = true; //After that, it will not be a initialization
//}
//
//void KalmanFilter::update(double dt, double distance, double angle){
//
//    if(!initialized)
//        throw std::runtime_error("Filter is not initialized!"); //Skip everything - initialization had failed
//
//    Eigen::VectorXd Obs(m);  // Observations: receive in polar and transform them to cart
//    Obs << distance * cos(angle), distance * sin(angle);
//
//    //Populate matrices
//    Q <<    pow(dt,4)/4, pow(dt,3)/2, 0, 0,
//    pow(dt,3)/2, pow(dt,2),   0, 0,
//    0, 0,                     pow(dt,4)/4, pow(dt,3)/2,
//    0, 0,                     pow(dt,3)/2, pow(dt,2);
//
//    F << 1, dt, 0, 0, 0, 1, 0, 0, 0, 0, 1, dt, 0, 0, 0, 1;
//
//    //Predict
//    estimatedCandidate = F * estimatedState;
//
//    /*
//     std::cout << std::fixed << std::showpoint;
//     std::cout << std::setprecision(4);
//     std::cout << "Update(estimatedCandidate): " << estimatedCandidate << endl;
//     std::cout << "Update(obs): " << Obs << endl;
//     std::cout << "Update(Q): " << Q << endl;
//     std::cout << "Update(F): " << F << endl;
//     */
//
//
//    candidateP = F * P * F.transpose() + Q;
//
//    /*
//     std::cout << "Update(candP): " << candidateP << endl;
//     */
//
//    //Calculate Kalman gain
//    K = candidateP * H.transpose() * ( H * candidateP * H.transpose() + R).inverse();
//
//    //Update
//    estimatedState = estimatedCandidate + K * (Obs - H * estimatedCandidate);
//
//    /*
//     std::cout << "Update(K): " << K << endl;
//     std::cout << "Update(estimatedState): " << estimatedState << endl;
//     */
//
//    P = (I - K * H) * candidateP;
//}
//
//
////Initial parameters
//const int _N = 4; //Number of states
//const int _M = 2; //Number of measurements
//
////double sigma_motion = 0.5; //Motion uncertainty [m]
//const double _sigma_distance = 0.5;         //Error of sensor in distance[m]
//const double _sigma_theta    = 0.0349066;   //Error of sensor in angle [rad]
//
//
//static double dt; // Delta of time steps
//static Eigen::MatrixXd F(_N, _N); // System dynamics matrix
//static Eigen::MatrixXd H(_M, _N); // Output matrix
//static Eigen::MatrixXd Q(_N, _N); // Process noise covariance
//static Eigen::MatrixXd R(_M, _M); // Measurement noise covariance
//static Eigen::MatrixXd P(_N, _N); // Estimate error covariance
//static Eigen::MatrixXd candidateP(_N, _N); // Estimate error covariance of candidate
//
//static KalmanFilter * m_kf_ptr = NULL;
//
//static double m_time_acc = 1;
//
//void kalmanInit(void)
//{
//    //Discrete linear motion of target, measuring position and speed on X and Y axis
//    //Static data attribution
//    H << 1, 0, 0, 0, 0, 0, 1, 0;
//    R << _sigma_distance, 0, _sigma_theta, 0;
//
//    //Initial covariance matrix
//    double kalmanPx  = 1.5;   //position error on x-axis [m]
//    double kalmanPvx = 28.0;  //speed error on x-axis [m/s]
//    double kalmanPy  = 0.3;   //position error on y-axis [m]
//    double kalmanPvy = 8.0;   //speed error on y-axis [m/s]
//    P << kalmanPx, 0, 0, 0, 0, kalmanPvx, 0, 0, 0, 0, kalmanPy, 0, 0, 0, 0, kalmanPvy;
//
//    // Construct the filter
//    if (m_kf_ptr)
//        delete m_kf_ptr;
//
//    m_kf_ptr = new KalmanFilter(0, F, H, Q, R, P, candidateP);
//
//    //Initial state - guess based
//    Eigen::VectorXd x0(_N);
//    double x_ini  = 10.63;   //Simulated target position on X axis [m]
//    double vx_ini = 24.16;   //Simulated target speed on X axis [m/s]
//    double y_ini  = 2.83;    //Simulated target position on Y axis [m]
//    double vy_ini = 6.43;    //Simulated target speed on Y axis [m/s]
//
//    x0 << x_ini, vx_ini, y_ini, vy_ini;
//
//    //Dynamic data attribution, data feeding into filter, output estimated states
//    m_time_acc = 1; //Overall time
//
//    m_kf_ptr->init(m_time_acc, x0);
//
//    //Visualization
//    std::cout << std::fixed << std::showpoint;
//    std::cout << std::setprecision(4);
//    std::cout << "timestep: " << m_time_acc << ", " << "distance[m]: " << m_kf_ptr->distanceRet() <<
//    ", " << "angle [rad]:" << m_kf_ptr->angleRet() << std::endl;
//}
//
////---- update for each measurement
//void kalmanUpdate(double deltaTime, double distance, double angle)
//{
//    if ( !m_kf_ptr )
//        return;
//
//    if (deltaTime == 0)
//        deltaTime = 0.040;
//
//    /*
//     std::cout << std::fixed << std::showpoint;
//     std::cout << std::setprecision(4);
//     std::cout << "Angle: " <<  angle << "distance: " << distance << endl;
//     std::cout << "EstimatedState: " << m_kf_ptr->getEstimatedState() << endl <<
//     "CovMatix: " << m_kf_ptr->getCovMatrix() << endl;
//     */
//
//    m_kf_ptr->update(deltaTime, distance, angle);
//    m_time_acc += deltaTime; //Overall counter
//
//    std::cout << std::fixed << std::showpoint;
//    std::cout << std::setprecision(4);
//    std::cout << "timestep: " << m_time_acc << ", " << "distance[m]: " << m_kf_ptr->distanceRet() <<
//    ", " << "angle [rad]:" << m_kf_ptr->angleRet() << std::endl;
//}
//
////---- return speed and accuracy
//double kalmanFinish()
//{
//    if ( !m_kf_ptr )
//        return 0;
//
//    double speed = m_kf_ptr->getSpeed();
//
//    std::cout << std::fixed << std::showpoint;
//    std::cout << std::setprecision(4);
//    std::cout << "Finish speed is: " << speed << std::endl;
//
//    delete m_kf_ptr;
//    m_kf_ptr = NULL;
//
//    return speed;
//}
//


#include "kalman_filter.h"
#include "tools.h"

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
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    // Kalman filter update step. Equations from the lectures
    VectorXd y_ = z - H_ * x_; // error calculation
    
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * Ht * Si;
    // New state
    x_ = x_ + (K * y_);
    int x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
//
//    /* KF Measurement update */
//    y_ = z - H_ * x_;
    
//    MatrixXd Ht = H_.transpose();
//    MatrixXd S = H_ * P_ * Ht + R_;
//    MatrixXd Si = S.inverse();
//    MatrixXd K =  P_ * Ht * Si;
//    // New state
//    x_ = x_ + (K * y);
//    int x_size = x_.size();
//    MatrixXd I = MatrixXd::Identity(x_size, x_size);
//    P_ = (I - K * H_) * P_;
//
//    S_ = H_ * P_ * H_.transpose() + R_;
//    K_ =  P_ * H_.transpose() * S_.inverse();
//
//    //new state
//    x_ = x_ + (K_ * y_);
//    MatrixXd I = MatrixXd::Identity(4, 4);
//    P_ = (I_ - K_ * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /* Update the state by using Extended Kalman Filter equations */
    double rho = sqrt(pow(x_[0], 2) + pow(x_[1], 2));
    double theta = atan(x_(1) / x_(0));
    double rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
    VectorXd h = VectorXd(3); // h(x_)
    h << rho, theta, rho_dot;
    
    VectorXd y_ = z - h;
    
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * Ht * Si;
    // New state
    x_ = x_ + (K * y_);
    int x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
    
//    S_ = H_ * P_ * H_.transpose() + R_;
//    K_ =  P_ * H_.transpose() * S_.inverse();
//
//    //new state
//    x_ = x_ + (K_ * y_);
//    //long x_size = x_.size();
//    MatrixXd I = MatrixXd::Identity(4, 4);
//    P_ = (I_ - K_ * H_) * P_;
}







