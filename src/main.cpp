#include <iostream>
#include "helperFunc.h"

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
    
    // print some dataset variables to check that they have been parsed properly
    std::cout << "elevInput at index 0: " << elevInput(0) << std::endl;
    std::cout << "elevInput at index 1: " << elevInput(1) << std::endl;
    std::cout << "elevInput at index 1000: " << elevInput(1000) << std::endl;
    std::cout << "pitchRateMeasured at index 0: " << pitchRateMeasured(0) << std::endl;
    std::cout << "pitchRateMeasured at index 1: " << pitchRateMeasured(1) << std::endl;
    std::cout << "pitchRateMeasured at index 1000: " << pitchRateMeasured(1000) << std::endl;
}