#include <iostream>
#include "EKF.h"

EKF::EKF(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
        const Eigen::MatrixXd &P0, const double dt) : Q_(Q), 
        R_(R), P0_(P0), sample_time_(dt)
{
    // Initialize dynamic system matrices
    A_.resize(num_state_, num_state_);
    A_ << param.Xu, param.Xw, 0        , -param.g * cos(param.theta0),
          param.Zu, param.Zw, param.U0 , -param.g * sin(param.theta0),
          param.Mu, param.Mw, param.Mq , 0,
          0       , 0       , 1        , 0;

    B_.resize(num_state_, num_input_);
    B_ << param.Xde, param.Xdth,
          param.Zde, 0,
          param.Mde, param.Mdth,
          0        , 0;

    C_.resize(num_output_, num_state_);
    C_ << 0, 1 / param.U0, 0, 0,
          0, 0           , 1, 0;

    I_.setIdentity(num_state_, num_state_);    
    
    std::cout << "EKF object has been constructed" << std::endl;
}

void EKF::Init(const Eigen::VectorXd &x0)
{
    x_hat_ = x0;
    P_ = P0_;
    time_ = 0.0;
    std::cout << "EKF object has been initialized" << std::endl;
}

void EKF::EvaluateLinearizedSys()
{
    A_(0, 3) = -param.g * cos(x_hat_(3));
    A_(1, 2) = x_hat_(0);
    A_(1, 3) = -param.g * sin(x_hat_(3));
    C_(0, 1) = 1 / x_hat_(0);
}

void EKF::Update(const Eigen::VectorXd &y, const Eigen::VectorXd &u, const Eigen::VectorXd &x)
{
    // Evaluate the linearized system matrices based on the updated state from the integrator
    x_hat_ = x;
    EKF::EvaluateLinearizedSys();

    // Predict the estimate error covariance matrix
    P_ = A_ * P_ * A_.transpose() + Q_;

    // Correction step
    K_ = P_ * C_.transpose() * (C_ * P_ * C_.transpose() + R_).inverse();
    x_hat_ += K_ * (y - C_ * x_hat_);
    P_ =  (I_ - K_ * C_) * P_;

    // Prepare for next time step
    time_ += sample_time_;
}

Eigen::VectorXd EKF::GetState()
{
    return x_hat_;
}

Eigen::VectorXd EKF::GetEstimatedOutput()
{
    return C_ * x_hat_;
}

double EKF::GetTime()
{
    return time_;
}
