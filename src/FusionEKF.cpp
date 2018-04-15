#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

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
        cout << "EKF state init: " << endl;
        cout << ekf_.x_ << endl;
        cout << "EKF covariance init: " << endl;
        cout << ekf_.P_ << endl;
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
    *  Prediction
    ****************************************************************************/
    // Calculate the timestep between measurements in seconds
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;    //dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;
    
    // State transition matrix update
    ekf_.F_ = MatrixXd(4, 4);
    // It is necessary to setup matrix F all the time, because delta_t may vary
    ekf_.F_ <<  1, 0, dt, 0,
                0, 1, 0, dt,
                0, 0, 1, 0,
                0, 0, 0, 1;
    
    // Noise covariance matrix computation
    float noise_ax = 9.0;
    float noise_ay = 9.0;
    float dt_2 = dt * dt; //dt^2
    float dt_3 = dt_2 * dt; //dt^3
    float dt_4 = dt_3 * dt; //dt^4
    float dt_4_4 = dt_4 / 4; //dt^4/4
    float dt_3_2 = dt_3 / 2; //dt^3/2
    
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ <<  dt_4_4 * noise_ax,  0,                  dt_3_2 * noise_ax,  0,
                0,                  dt_4_4 * noise_ay,  0,                  dt_3_2 * noise_ay,
                dt_3_2 * noise_ax,  0,                  dt_2 * noise_ax,    0,
                0,                  dt_3_2 * noise_ay,  0,                  dt_2 * noise_ay;
    
    // Debug by print
    //cout << "F: " << endl;
    //cout << ekf_.F_ << endl;
    //cout << "Q: " << endl;
    //cout << ekf_.Q_ << endl;
    
    ekf_.Predict();
    /* print the output */
    std::cout << "Predict state x_ = " << endl;
    std::cout << ekf_.x_ << endl;
    std::cout << "Predict covariance P_ = " << endl;
    std::cout << ekf_.P_ << endl;
    
    /*****************************************************************************
    *  Update
    ****************************************************************************/
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.R_ = R_radar_;
        std::cout << "Update step radar H_ = " << endl;
        std::cout << ekf_.H_ << endl;
        std::cout << "Update step radar R_ = " << endl;
        std::cout << ekf_.R_ << endl;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
        std::cout << "Update step x_ = " << endl;
        std::cout << ekf_.x_ << endl;
        std::cout << "Update step covariance P_ = " << endl;
        std::cout << ekf_.P_ << endl;
    } else {
        // Laser updates
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
        std::cout << "Update step laser H_ = " << endl;
        std::cout << ekf_.H_ << endl;
        std::cout << "Update step laser R_ = " << endl;
        std::cout << ekf_.R_ << endl;
        ekf_.Update(measurement_pack.raw_measurements_);
        std::cout << "Update step x_ = " << endl;
        std::cout << ekf_.x_ << endl;
        std::cout << "Update step covariance P_ = " << endl;
        std::cout << ekf_.P_ << endl;
    }

    /* print the output */
    cout << "x_ = " << endl;
    cout << ekf_.x_ << endl;
    cout << "P_ = " << endl;
    cout << ekf_.P_ << endl;
    //cout << "dt = " << endl;
    //cout << ekf_.P_ << endl;
}
