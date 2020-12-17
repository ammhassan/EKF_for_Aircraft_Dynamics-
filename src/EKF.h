#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>
#include "LongDynamics.h"

class EKF
{
    public:
    // Constructor
    EKF(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
        const Eigen::MatrixXd &P0, const double dt);

    // Linearized system evaluation
    void EvaluateLinearizedSys();

    // Filter update function, y: output measurement vector, u: control input vector 
    void Update(const Eigen::VectorXd &y, const Eigen::VectorXd &u, const Eigen::VectorXd &x);

    // Getters
    Eigen::VectorXd GetState();
    Eigen::VectorXd GetEstimatedOutput();
    double GetTime();           

    private:
    // Dimensions of the dynamic system
    const int num_state_ = 4;
    const int num_input_ = 2;
    const int num_output_ = 2;

    // Time and sampleTime
    double sample_time_;

    // Dynamic system matrices
    Eigen::MatrixXd A_; // Dynamics matrix
    Eigen::MatrixXd B_; // Control input matrix
    Eigen::MatrixXd C_; // Output matrix
    Eigen::MatrixXd Q_; // Process noise covariance matrix
    Eigen::MatrixXd R_; // Measurment noise covariance matrix

    // Filter matrices
    Eigen::MatrixXd P_; // Estimate error covariance matrix
    Eigen::MatrixXd P0_; // Initial value for P
    Eigen::MatrixXd K_; // Kalman gain matrix
    Eigen::MatrixXd I_; // identity matrix

    // State estimate at the current time step
    Eigen::VectorXd x_hat_;
};

#endif