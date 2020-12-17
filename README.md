# Extended Kalman Filter for Aircraft Dynamics

The goal of this project is to implement an extended Kalman filter (EKF) to estimate
a fixed-wing aicraft state vector from noisy sensor measurments.
The first iteration of this project will be focused only on the longitudinal dynamics.
Full documentation is available in `doc/EKF_for_Aircraft_Dynamics.pdf`.

## Dependencies for Running Locally
* CMake: v3.7 or higher
* Make: v4.1 or higher
* gcc/g++: v5.4 or higher
* Eigen: v3.2.10 or higher
  * [Click here for installation instructions](http://eigen.tuxfamily.org/dox/GettingStarted.html)
* odeint: v2.2 or higher
  * [Click here for installation instructions](http://headmyshoulder.github.io/odeint-v2/doc/index.html)

## Basic Build Instructions

1. Clone this repo
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./EKF_test`

## Expected Behavior
The expected output from this program looks like this: <br />
`Dataset has been loaded` <br />
`EKF object has been constructed` <br />
`EKF object has been initialized` <br />
`Processing dataset...` <br />
`Finished processing dataset` <br />
`RMSE for the estimated AoA is: 0.000735383` <br />
`RMSE for the estimated pitch rate is: 0.000681878` <br />