#include "helper_func.h"
#include <fstream>
#include <iostream>
#include <math.h>

// This function loads a dataset of I/O from a text file.
void LoadDataSet(std::string path, Eigen::VectorXd &elev_input, Eigen::VectorXd &throttle_input,
                 Eigen::VectorXd &alpha_true, Eigen::VectorXd &alpha_measured, 
                 Eigen::VectorXd &pitch_rate_true, Eigen::VectorXd &pitch_rate_measured) 
{
  std::string line;
  std::string time;
  double de, dt, a_true, a_measured, q_true, q_measured;
  std::ifstream filestream(path);
  if (filestream.is_open()) 
  {
    int i = 0;
    while (std::getline(filestream, line)) 
    {
      std::istringstream linestream(line);
      while (linestream >> time >> de >> dt >> a_true >> a_measured >> q_true >> q_measured) {
        if (time == "time") 
        {
          // first line that has the columns titles - do not parse data
        }
        else 
        {
            elev_input(i) = de;
            throttle_input(i) = dt;
            alpha_true(i) = a_true;
            alpha_measured(i) = a_measured;
            pitch_rate_true(i) = q_true;
            pitch_rate_measured(i) = q_measured;
            ++i;
        }
      }
    }
    std::cout << "Dataset has been loaded" << std::endl;
  }
} // void loadDataSet


// This function computes the root mean square error between the true output and the estimated one.
float ComputeRMSE(Eigen::VectorXd &true_output, Eigen::VectorXd &estimated_output)
{
    float meanSquareError = 0.0;
    for (int i = 0; i < true_output.size(); ++i)
    {
        double error = true_output(i) - estimated_output(i);
        meanSquareError += error * error / true_output.size();
    }
return sqrt(meanSquareError);
}