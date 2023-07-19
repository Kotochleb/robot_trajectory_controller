#ifndef __ROBOT_DYNAMICS_HPP__
#define __ROBOT_DYNAMICS_HPP__

#include "IpTypes.hpp"
#include "cmath"

namespace robot_dynamics {

class RobotDynamics {
 public:
  RobotDynamics(const double wheel_separation, const double wheel_radius);
  void simulate(const Ipopt::Number* control, const Ipopt::Number* x0,
                const std::size_t steps);

 private:
  inline void rk4_step(const Ipopt::Number* x0, const Ipopt::Number* u,
                       Ipopt::Number* xk1);
  inline void sim_step(const Ipopt::Number* x0, const Ipopt::Number* u,
                       Ipopt::Number* x_dot);
  inline void add_vectors(const Ipopt::Number* x0, const Ipopt::Number* u,
                          Ipopt::Number* x_dot);

};  // namespace robot_dynamics

#endif  // __ROBOT_DYNAMICS_HPP__