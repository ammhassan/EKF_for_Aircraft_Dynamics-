#ifndef HELPER_FUNC_H
#define HELPER_FUNC_H

#include <Eigen/Dense>

void LoadDataSet(std::string path, Eigen::VectorXd &elev_input, Eigen::VectorXd &throttle_input,
                 Eigen::VectorXd &alpha_true, Eigen::VectorXd &alpha_measured, 
                 Eigen::VectorXd &pitch_rate_true, Eigen::VectorXd &pitch_rate_measured);

float ComputeRMSE(Eigen::VectorXd &true_output, Eigen::VectorXd &estimated_output);     

#endif