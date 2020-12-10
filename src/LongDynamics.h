#ifndef LONG_DYNAMICS_H
#define LONG_DYNAMICS_H

#include <iostream>
#include <vector>
#include <math.h>
#include <boost/numeric/odeint.hpp>

#define PI 3.14159265

typedef std::vector<double> state_type;

// The rhs of x' = f(x, u) defined as a class
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

        dxdt[0] = -x[2] * x[1] - g * cos(theta0) * (x[3] - theta0) + Xu * (x[0] - U0)
                + Xw * (x[1] - W0) + Xde * de + Xdth * dth;
        dxdt[1] = x[2] * x[0] - g * sin(theta0) * (x[3] - theta0) + Zu * (x[0] - U0)
                + Zw * (x[1] - W0) + Zde * de;
        dxdt[2] = Mu * (x[0] - U0) + Mw * (x[1] - W0) + Mq * x[2] + Mde * de + Mdth * dth;
        dxdt[3] = x[2];
    }

private:
    // Aircraft parameters
    static constexpr uint mass = 300000;
    static constexpr float g = 9.81;
    static constexpr double Xu = -0.02;
    static constexpr double Xw = 0.1;
    static constexpr double Zu = -0.23;
    static constexpr double Zw = -0.634;
    static constexpr double Mu = -0.0000255;
    static constexpr double Mw = -0.005;
    static constexpr double Mq = -0.61;
    static constexpr double Xde = 0.14;
    static constexpr double Zde = -2.9;
    static constexpr double Mde = -0.64;
    static constexpr double Xdth = 1.56;
    static constexpr double Mdth = 0.0054;

    // Initial condition
    static constexpr double theta0 = 2.7 * PI / 180;
    static constexpr double alpha0 = theta0;
    static constexpr double U0 = 74.9167;
    static constexpr double W0 = 3.533;
};

#endif