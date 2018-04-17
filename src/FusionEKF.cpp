#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define DEBUG   0

/* Constructor */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices and vectors
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;
    
    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
                0,      0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09,   0,      0,
                0,      0.0009, 0,
                0,      0,      0.09;      
}
    
/* Destructor */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        // first measurement
        cout << "EKF starts " << endl;
        
        ekf_.x_ = VectorXd(4);
        ekf_.Q_ = MatrixXd(4,4);
        ekf_.P_ = MatrixXd(4,4);
        
        /* Initial covariance matrix */
        ekf_.P_ <<  1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1000, 0,
                    0, 0, 0, 1000;
        
        //Start time counting
        previous_timestamp_ = measurement_pack.timestamp_;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /* Convert radar from polar to cartesian coordinates and initialize state. */
            double rho = measurement_pack.raw_measurements_[0];     // range
            double phi = measurement_pack.raw_measurements_[1];     // bearing
            double rho_dot = measurement_pack.raw_measurements_[2]; // velocity of rho
            double x = rho * cos(phi);
            double y = rho * sin(phi);
            double vx = rho_dot * cos(phi);
            double vy = rho_dot * sin(phi);
            ekf_.x_ << x, y, vx , vy;
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;  /* Initialize state. */
        }
        
        // Print init states
        if (DEBUG == 1){
            cout << "EKF state init: " << endl;
            cout << ekf_.x_ << endl;
            cout << "EKF covariance init: " << endl;
            cout << ekf_.P_ << endl;
        }
        
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
    *  Prediction
    ****************************************************************************/
    // Calculate the timestep between measurements in seconds
    float delta_t = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;    //dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;
    
    // State transition matrix update
    ekf_.F_ = MatrixXd(4, 4);
    // It is necessary to setup matrix F all the time, because delta_t may vary
    ekf_.F_ <<  1, 0, delta_t, 0,
                0, 1, 0, delta_t,
                0, 0, 1, 0,
                0, 0, 0, 1;
    
    // Noise covariance matrix computation
    float noise_ax = 9.0;
    float noise_ay = 9.0;
    float delta_t_squared = delta_t * delta_t; //dt^2
    float delta_t_cubic = delta_t_squared * delta_t; //dt^3
    float delta_t_pow4 = delta_t_cubic * delta_t; //dt^4
    float delta_t_pow4_by4 = delta_t_pow4 / 4; //dt^4/4
    float delta_t_cubic_by2 = delta_t_cubic / 2; //dt^3/2
    
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ <<  delta_t_pow4_by4 * noise_ax,  0,                  delta_t_cubic_by2 * noise_ax,  0,
                0,                  delta_t_pow4_by4 * noise_ay,  0,                  delta_t_cubic_by2 * noise_ay,
                delta_t_cubic_by2 * noise_ax,  0,                  delta_t_squared * noise_ax,    0,
                0,                  delta_t_cubic_by2 * noise_ay,  0,                  delta_t_squared * noise_ay;
    
    // Debug by print
    if (DEBUG == 1){
        cout << "Prediction step F: " << endl;
        cout << ekf_.F_ << endl;
        cout << "Prediction step Q: " << endl;
        cout << ekf_.Q_ << endl;
    }
    
    ekf_.Predict();
    /* print the output */
    if (DEBUG == 1){
        cout << "Predicted state x_ = " << endl;
        cout << ekf_.x_ << endl;
        cout << "Predicted covariance P_ = " << endl;
        cout << ekf_.P_ << endl;
    }
    
    
    /*****************************************************************************
    *  Update
    ****************************************************************************/
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.R_ = R_radar_;
        if (DEBUG == 1){
            cout << "Update step radar H_ = " << endl;
            cout << ekf_.H_ << endl;
            cout << "Update step radar R_ = " << endl;
            cout << ekf_.R_ << endl;
        }
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
        if (DEBUG == 1){
            cout << "Update step x_ = " << endl;
            cout << ekf_.x_ << endl;
            cout << "Update step covariance P_ = " << endl;
            cout << ekf_.P_ << endl;
        }
    }
    else
    {
        // Laser updates
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
        if (DEBUG == 1){
            cout << "Update step laser H_ = " << endl;
            cout << ekf_.H_ << endl;
            cout << "Update step laser R_ = " << endl;
            cout << ekf_.R_ << endl;
        }
        ekf_.Update(measurement_pack.raw_measurements_);
        if (DEBUG == 1){
            cout << "Update step x_ = " << endl;
            cout << ekf_.x_ << endl;
            cout << "Update step covariance P_ = " << endl;
            cout << ekf_.P_ << endl;
        }
    }

    /* print the output */
    if (DEBUG == 1){
        cout << "x_ = " << endl;
        cout << ekf_.x_ << endl;
        cout << "P_ = " << endl;
        cout << ekf_.P_ << endl;
    }
}
