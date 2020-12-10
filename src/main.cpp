#include <iostream>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include "helper_func.h"
#include "LongDynamics.h"

int main()
{
    // define the dataset path
    const std::string dataSetPath{"../data/aircraft_long_dynamics_dataset.txt"};

    // load data set
    int dataSetSize = 1001;
    Eigen::VectorXd elevInput(dataSetSize);
    Eigen::VectorXd throttleInput(dataSetSize);
    Eigen::VectorXd alphaTrue(dataSetSize);
    Eigen::VectorXd alphaMeasured(dataSetSize);
    Eigen::VectorXd pitchRateTrue(dataSetSize);
    Eigen::VectorXd pitchRateMeasured(dataSetSize);
    loadDataSet(dataSetPath, elevInput, throttleInput, alphaTrue, 
                alphaMeasured, pitchRateTrue, pitchRateMeasured);
    
    // Print some dataset variables to check that they have been parsed properly
    std::cout << "elevInput at index 0: " << elevInput(0) << std::endl;
    std::cout << "elevInput at index 1: " << elevInput(1) << std::endl;
    std::cout << "elevInput at index 1000: " << elevInput(1000) << std::endl;
    std::cout << "pitchRateMeasured at index 0: " << pitchRateMeasured(0) << std::endl;
    std::cout << "pitchRateMeasured at index 1: " << pitchRateMeasured(1) << std::endl;
    std::cout << "pitchRateMeasured at index 1000: " << pitchRateMeasured(1000) << std::endl;


    // State_initialization
    state_type x(4);
    x[0] = 74.9167;
    x[1] = 3.5330;
    x[2] = 0.0;
    x[4] = 2.7 * PI / 180;

    // Integration_class
    LongDynamics eomObject;

    // Define_const_stepper
    boost::numeric::odeint::runge_kutta4<state_type> stepper;

    // Print intial state
    std::cout << "U at t0: " << x[0] << std::endl;
    std::cout << "W at t0: " << x[1] << std::endl;
    std::cout << "Q at t0: " << x[2] << std::endl;
    std::cout << "theta at t0: " << x[3] << std::endl;

    // Integrate_const_loop
    const double dt = 0.01;
    for(double t = 0.0 ; t < 10.0 ; t += dt)
    {
        stepper.do_step(eomObject, x, t, dt);
    }

    // Print final state
    std::cout << "U at tf: " << x[0] << std::endl;
    std::cout << "W at tf: " << x[1] << std::endl;
    std::cout << "Q at tf: " << x[2] << std::endl;
    std::cout << "theta at tf: " << x[3] << std::endl;

}