#include <iostream>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include "helper_func.h"
#include "LongDynamics.h"
#include "EKF.h"

int main()
{
    // Define the dataset path
    const std::string dataset_path{"../data/aircraft_long_dynamics_dataset.txt"};

    // Load data set
    const int dataset_size = 1001;
    Eigen::VectorXd elev_input(dataset_size);
    Eigen::VectorXd throttle_input(dataset_size);
    Eigen::VectorXd alpha_true(dataset_size);
    Eigen::VectorXd alpha_measured(dataset_size);
    Eigen::VectorXd pitch_rate_true(dataset_size);
    Eigen::VectorXd pitch_rate_measured(dataset_size);
    LoadDataSet(dataset_path, elev_input, throttle_input, alpha_true, 
                alpha_measured, pitch_rate_true, pitch_rate_measured);
    
    // Define dynamic system parameters
    int num_state = 4;
    int num_input = 2;
    int num_output = 2;
    const double sample_time = 0.01;
    Eigen::MatrixXd Q(num_state, num_state); // Process noise covariance matrix
    Q << 0.0001, 0.0   , 0.0     , 0.0,
         0.0   , 0.0001, 0.0     , 0.0,
         0.0   , 0.0   , 0.000001, 0.0,
         0.0   , 0.0   , 0.0     , 0.0;

    Eigen::MatrixXd R(num_output, num_output); // Measurment noise covariance matrix
    R << 0.00001, 0.0,
         0.0    , 0.00001;

    Eigen::MatrixXd P0(num_state, num_state); // Initial estimate error covariance matrix
    P0 << 0.0001, 0.0   , 0.0     , 0.0,
         0.0   , 0.0001, 0.0     , 0.0,
         0.0   , 0.0   , 0.000001, 0.0,
         0.0   , 0.0   , 0.0     , 0.0;

    // Construct EKF object
    EKF ekf(Q, R, P0, sample_time);

    // Initial state for the filter
    Eigen::VectorXd x0(num_state);
    x0 << 74.9167 + 0.03,
          3.5330 + 0.03,
          0.0 +0.001,
          2.7 * PI / 180;

    // Define integration class and a stepper integrator
    state_type x(num_state);
    LongDynamics long_dyn_object;
    boost::numeric::odeint::runge_kutta4<state_type> rk4_stepper;

    // Define vectors to hold the estimated outputs and the integrator predicted state
    Eigen::VectorXd alpha_estimated(dataset_size);
    Eigen::VectorXd pitch_rate_estimated(dataset_size);
    Eigen::VectorXd predicted_state(num_state);

    // Initialize the state
    predicted_state << 74.9167 + 0.03,
                       3.5330 + 0.03,
                       0.0 +0.001,
                       2.7 * PI / 180;

    // Run the filter on the data set of I/O
    Eigen::VectorXd y(num_output);
    Eigen::VectorXd u(num_input);
    double t = 0.0;
    std::cout << "Processing dataset..." << std::endl;
    for (int i = 0; i < dataset_size; i++)
    {
        // Run EKF for one step
        y << alpha_measured(i),
             pitch_rate_measured(i);
        u << elev_input(i),
             throttle_input(i);
        ekf.Update(y, u, predicted_state);
        alpha_estimated(i) = ekf.GetEstimatedOutput()(0);
        pitch_rate_estimated(i) = ekf.GetEstimatedOutput()(1);

        // Pass the updated state back to the integrator
        x[0] = ekf.GetState()(0);
        x[1] = ekf.GetState()(1);
        x[2] = ekf.GetState()(2);
        x[3] = ekf.GetState()(3);
        
        // Run the integrator for one step and get the new state
        t = sample_time * i;
        rk4_stepper.do_step(long_dyn_object, x, t, sample_time);
        predicted_state(0) = x[0];
        predicted_state(1) = x[1];
        predicted_state(2) = x[2];
        predicted_state(3) = x[3];       
    }
    std::cout << "Finished processing dataset" << std::endl;

    // Compute and print root mean square error for the estimated outputs (angle of attack and pitch rate)
    std::cout << "RMSE for the estimated AoA is: " << ComputeRMSE(alpha_true, alpha_estimated) << std::endl;
    std::cout << "RMSE for the estimated pitch rate is: " << ComputeRMSE(pitch_rate_true, pitch_rate_estimated) << std::endl;
}