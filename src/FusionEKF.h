#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
    public:
        /* Constructor */
        FusionEKF();

        /* Destructor */
        virtual ~FusionEKF();

        /**
         * Run the whole flow of the Kalman Filter from here.
         */
        void ProcessMeasurement(const MeasurementPackage &measurement_pack);

        /**
         * Kalman Filter update and prediction math lives in here.
         */
        KalmanFilter ekf_;

    private:
        // check whether the tracking toolbox was initialized or not (first measurement)
        bool is_initialized_;

        // previous timestamp
        long long previous_timestamp_;

        // tool object used to compute Jacobian and RMSE
        Tools tools;
        Eigen::MatrixXd R_laser_;
        Eigen::MatrixXd R_radar_;
        Eigen::MatrixXd H_laser_;
        Eigen::MatrixXd Hj_;
    
        // Kalman filter variables
        //Eigen::VectorXd x_;  // State space vector
        //Eigen::VectorXd z_;  // Observation vector
        //Eigen::VectorXd y_;  // Observation prediction
        //Eigen::MatrixXd S_;
        //Eigen::MatrixXd K_;  // Kalman gain
        //Eigen::MatrixXd P_;  // Covariance matrix
    
        float noise_ax, noise_ay;
};

#endif /* FusionEKF_H_ */
