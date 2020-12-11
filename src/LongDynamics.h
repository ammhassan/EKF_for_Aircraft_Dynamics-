#ifndef LONG_DYNAMICS_H
#define LONG_DYNAMICS_H

#include <iostream>
#include <vector>
#include <math.h>
#include <boost/numeric/odeint.hpp>

#define PI 3.14159265

// Define the aircraft parameters
static const struct AircraftParam {
    const uint mass = 300000;
    const float g = 9.81;
    const double Xu = -0.02;
    const double Xw = 0.1;
    const double Zu = -0.23;
    const double Zw = -0.634;
    const double Mu = -0.0000255;
    const double Mw = -0.005;
    const double Mq = -0.61;
    const double Xde = 0.14;
    const double Zde = -2.9;
    const double Mde = -0.64;
    const double Xdth = 1.56;
    const double Mdth = 0.0054;
} param;

// The rhs of x' = f(x, u) defined as a class
typedef std::vector<double> state_type;
class LongDynamics {
public:
    LongDynamics() {}

    void operator() (const state_type &x , state_type &dxdt , const double  t)
    {
        // x[0]: U
        // x[1]: W
        // x[2]: Q
        // x[3]: theta

        double dth = 0;
        double de = 5 * PI / 180 * sin(PI * t);

        dxdt[0] = -x[2] * x[1] - param.g * cos(theta0_) * (x[3] - theta0_) + param.Xu * (x[0] - U0_)
                + param.Xw * (x[1] - W0_) + param.Xde * de + param.Xdth * dth;
        dxdt[1] = x[2] * x[0] - param.g * sin(theta0_) * (x[3] - theta0_) + param.Zu * (x[0] - U0_)
                + param.Zw * (x[1] - W0_) + param.Zde * de;
        dxdt[2] = param.Mu * (x[0] - U0_) + param.Mw * (x[1] - W0_) + param.Mq * x[2] 
                + param.Mde * de + param.Mdth * dth;
        dxdt[3] = x[2];
    }

private:
    // Initial condition
    static constexpr double theta0_ = 2.7 * PI / 180;
    static constexpr double alpha0_ = theta0_;
    static constexpr double U0_ = 74.9167;
    static constexpr double W0_ = 3.533;
};

#endif