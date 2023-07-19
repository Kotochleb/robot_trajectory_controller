#include "robot_dynamics.hpp"

#include <cmath>

namespace robot_dynamics {

void RobotDynamics::simulate(const Ipopt::Vector& u, const Ipopt::Vector& x0,
                             const Ipopt::Vector* x_out,
                             const std::size_t steps) {
  for (std::size_t i = 0; i < steps; i++) {
    auto k1 = f(x0, u[i]);
    auto k2 = f(x0 + k1 * dt / 2.0, u[i]);
    auto k3 = f(x0 + k2 * dt / 2.0, u[i]);
    auto k4 = f(x0 + k3 * dt, u[i]);
    x_out[i] = X[:, i] + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
  }
}

inline void RobotDynamics::rk4_step(const Ipopt::Number* x0,
                                    const Ipopt::Number* u,
                                    Ipopt::Number* xk1) {
                                      
                                    }

inline void RobotDynamics::sim_step(const Ipopt::Number* x0,
                                    const Ipopt::Number* u,
                                    Ipopt::Number* x_dot) {
  x_dot[0] = x0[0] + u[0];
  x_dot[1] = x0[1] + x0[0] * sin(x0[4]);
  x_dot[2] = x0[2] + x0[0] * cos(x0[4]);
  x_dot[3] = x0[3] + u[2];
  x_dot[4] = x0[4] + x0[3];
}

};  // namespace robot_dynamics