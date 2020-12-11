# Extended Kalman Filter for Aircraft Dynamics

The goal of this project is to implement an extended Kalman filter (EKF) to estimate
a fixed-wing aicraft state vector from noisy sensor measurments.
The first iteration of this project will be focused only on the longitudinal dynamics.

## Dependencies for Running Locally
* cmake >= 3.7
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* Eigen >= 3.2.10
  * All OSes: [click here for installation instructions](http://eigen.tuxfamily.org/dox/GettingStarted.html)
* odeint >= 2.2
  * All OSes: [click here for installation instructions](http://headmyshoulder.github.io/odeint-v2/doc/index.html)
