#include "helper_func.h"
#include <fstream>
#include <iostream>
#include <math.h>

// This function loads a dataset of I/O from a text file.
void loadDataSet(std::string path, Eigen::VectorXd &elevInput, Eigen::VectorXd &throttleInput,
                 Eigen::VectorXd &alphaTrue, Eigen::VectorXd &alphaMeasured, 
                 Eigen::VectorXd &pitchRateTrue, Eigen::VectorXd &pitchRateMeasured) 
{
  std::string line;
  std::string time;
  double de, dt, aTrue, aMeasured, qTrue, qMeasured;
  std::ifstream filestream(path);
  if (filestream.is_open()) 
  {
    int i = 0;
    while (std::getline(filestream, line)) 
    {
      std::istringstream linestream(line);
      while (linestream >> time >> de >> dt >> aTrue >> aMeasured >> qTrue >> qMeasured) {
        if (time == "time") 
        {
          // first line that has the columns titles - do not parse data
        }
        else 
        {
            elevInput(i) = de;
            throttleInput(i) = dt;
            alphaTrue(i) = aTrue;
            alphaMeasured(i) = aMeasured;
            pitchRateTrue(i) = qTrue;
            pitchRateMeasured(i) = qMeasured;
            ++i;
        }
      }
    }
    std::cout << "Dataset has been loaded" << std::endl;
  }
} // void loadDataSet


// This function computes the root mean square error between the true output and the estimated one.
float computeRMSE(Eigen::VectorXd &trueOutput, Eigen::VectorXd &estimatedOutput)
{
    float meanSquareError = 0.0;
    for (int i = 0; i < trueOutput.size(); ++i)
    {
        double error = trueOutput(i) - estimatedOutput(i);
        meanSquareError += error * error / trueOutput.size();
    }
return sqrt(meanSquareError);
}