//#ifndef KALMAN_FILTER_H_
//#define KALMAN_FILTER_H_
//
//#include "Eigen/Dense"
//#include <cmath>
//
//#pragma once
//
//class KalmanFilter {
//
//public:
//
//    //Create a Kalman filter with the specified matrices.
//    //F - target motion model matrix
//    //H - Observation model matrix
//    //Q - Process noise covariance
//    //R - Measurement noise covariance
//    //P - Estimate error covariance
//    //candidateP - Estimate error covariance of prediction
//
//    KalmanFilter(
//                 double dt,
//                 const Eigen::MatrixXd& F,
//                 const Eigen::MatrixXd& H,
//                 const Eigen::MatrixXd& Q,
//                 const Eigen::MatrixXd& R,
//                 const Eigen::MatrixXd& P,
//                 const Eigen::MatrixXd& candidateP
//                 );
//
//    //Blank estimator
//    KalmanFilter();
//
//    //Filter initialization function
//    void init(double t0, const Eigen::VectorXd& x0);
//
//    //Predict and update next estimated state
//    void update(double dt, double distance, double angle);
//
//    //Return the current state in cartesian coordinates
//    Eigen::VectorXd state() {
//        return estimatedState;
//    };
//
//    //Return the current distance
//    double distanceRet(){
//        return sqrt( pow(estimatedState[1], 2) + pow(estimatedState[3], 2) );
//    }
//
//    //Return the current angle
//    double angleRet(){
//        return atan(estimatedState[3]/estimatedState[1]);
//    }
//
//    //Return final speed
//    double getSpeed() {
//        return sqrt( pow(estimatedState[2], 2) + pow(estimatedState[4], 2) );
//    }
//
//    Eigen::VectorXd getEstimatedState(){ return estimatedState; }
//    Eigen::MatrixXd getCovMatrix()     { return P; }
//
//    //Return time
//    double time() {
//        return t;
//    };
//
//private:
//
//    // Matrices for computation
//    Eigen::MatrixXd F, H, Q, R, P, candidateP, K, P0, Obs;
//
//    // System dimensions
//    int m, n;
//
//    // Initial and current time
//    double t0, t;
//
//    // Discrete time step
//    double dt;
//
//    // Is the filter initialized?
//    bool initialized;
//
//    // n-size identity
//    Eigen::MatrixXd I;
//
//    // Estimated states
//    Eigen::VectorXd estimatedState, estimatedCandidate;
//};
//
//void    kalmanInit(void);
//void    kalmanUpdate(double deltaTime, double distance, double angle);
//double  kalmanFinish();
//
//#endif /* KALMAN_FILTER_H_ */
//
//



#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
    public:
        // state vector
        Eigen::VectorXd x_;
        // measurement vector
        //Eigen::VectorXd z_;
        // state covariance matrix
        Eigen::MatrixXd P_;
        // state transition matrix
        Eigen::MatrixXd F_;
        // process covariance matrix
        Eigen::MatrixXd Q_;
        // measurement matrix
        Eigen::MatrixXd H_;
        // measurement covariance matrix
        Eigen::MatrixXd R_;
        //
        //Eigen::VectorXd y_;
        //
        //Eigen::MatrixXd S_;
        //
        //Eigen::MatrixXd K_;
        //
        //Eigen::MatrixXd Hj_;
        //
        //Eigen::MatrixXd I_;

        /* Constructor - blank estimator */
        KalmanFilter();

        /* Destructor */
        virtual ~KalmanFilter();

    /**
     * Init Initializes Kalman filter
     * @param x_in Initial state
     * @param P_in Initial state covariance
     * @param F_in Transition matrix
     * @param H_in Measurement matrix
     * @param R_in Measurement covariance matrix
     * @param Q_in Process covariance matrix
     */
    void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in, Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

    /**
     * Prediction Predicts the state and the state covariance
     * using the process model
     * @param delta_T Time between k and k+1 in s
     */
    void Predict();

    /**
     * Updates the state by using standard Kalman Filter equations
     * @param z The measurement at k+1
     */
    void Update(const Eigen::VectorXd &z);

    /**
     * Updates the state by using Extended Kalman Filter equations
     * @param z The measurement at k+1
     */
    void UpdateEKF(const Eigen::VectorXd &z);
};

#endif /* KALMAN_FILTER_H_ */

