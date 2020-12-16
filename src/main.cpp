#include <iostream>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include "helper_func.h"
#include "LongDynamics.h"
#include "EKF.h"

int main()
{
    // Define the dataset path
    const std::string dataSetPath{"../data/aircraft_long_dynamics_dataset.txt"};

    // Load data set
    int dataSetSize = 1001;
    Eigen::VectorXd elevInput(dataSetSize);
    Eigen::VectorXd throttleInput(dataSetSize);
    Eigen::VectorXd alphaTrue(dataSetSize);
    Eigen::VectorXd alphaMeasured(dataSetSize);
    Eigen::VectorXd pitchRateTrue(dataSetSize);
    Eigen::VectorXd pitchRateMeasured(dataSetSize);
    loadDataSet(dataSetPath, elevInput, throttleInput, alphaTrue, 
                alphaMeasured, pitchRateTrue, pitchRateMeasured);
    
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

    // Initialize the filter
    Eigen::VectorXd x0(num_state);
    x0 << 74.9167,
          3.5330,
          0.0,
          2.7 * PI / 180;
    ekf.Init(x0);


    // State_initialization for the integrator
    state_type x(num_state);
    x[0] = 74.9167;
    x[1] = 3.5330;
    x[2] = 0.0;
    x[3] = 2.7 * PI / 180;

    // Integration_class
    LongDynamics eomObject;

    // Define_const_stepper
    boost::numeric::odeint::runge_kutta4<state_type> stepper;

    // Define vectors to hold the estimated outputs and the integrator state
    Eigen::VectorXd alpha_estimated(dataSetSize);
    Eigen::VectorXd pitch_rate_estimated(dataSetSize);
    Eigen::VectorXd predicted_state(num_state);
    
    // Run the filter on the data set of I/O
    Eigen::VectorXd y(num_output);
    Eigen::VectorXd u(num_input);
    double t = 0.0;
    std::cout << "Processing dataset..." << std::endl;

    for (int i = 0; i < dataSetSize; i++)
    {
        // Run the integrator for one step and get the new state
        t = sample_time * i;
        stepper.do_step(eomObject, x, t, sample_time);
        predicted_state(0) = x[0];
        predicted_state(1) = x[1];
        predicted_state(2) = x[2];
        predicted_state(3) = x[3];

        // Run EKF for one step
        y << alphaMeasured(i),
             pitchRateMeasured(i);

        u << elevInput(i),
             throttleInput(i);

        ekf.Update(y, u, predicted_state);
        alpha_estimated(i) = ekf.GetEstimatedOutput()(0);
        pitch_rate_estimated(i) = ekf.GetEstimatedOutput()(1);
    }

    std::cout << "Finished" << std::endl;
    
    // Print final estimated outputs
    std::cout << "alpha at tf: " << alpha_estimated(dataSetSize-1) << std::endl;
    std::cout << "pitch rate at tf: " << pitch_rate_estimated(dataSetSize-1) << std::endl;
}