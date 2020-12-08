#include <Eigen/Dense>

void loadDataSet(std::string path, Eigen::VectorXd &elevInput, Eigen::VectorXd &throttleInput,
                 Eigen::VectorXd &alphaTrue, Eigen::VectorXd &alphaMeasured, 
                 Eigen::VectorXd &pitchRateTrue, Eigen::VectorXd &pitchRateMeasured);                 

