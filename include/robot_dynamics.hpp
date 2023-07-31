#ifndef __ROBOT_DYNAMICS_HPP__
#define __ROBOT_DYNAMICS_HPP__

// #include "IpTypes.hpp"
#include <cmath>

#include <iostream>

#include <Eigen/Dense>

namespace robot_dynamics {
using namespace Eigen;

template <class Derived, std::size_t state_variables, std::size_t control_inputs>
class RobotDynamics {
 public:
  typedef Matrix<float, state_variables, Dynamic> MatrixState;
  typedef Matrix<float, control_inputs, Dynamic> MatrixControl;
  typedef Matrix<float, state_variables, 1> VectorState;
  typedef Matrix<float, control_inputs, 1> VectorControl;
  
  RobotDynamics() {};
  
  void simulate(const MatrixControl& u, const VectorState& x0,
                MatrixState& x_out) {
    assert(u.rows() == x_out.rows());
    x_out.col(0) = x0;
    VectorState k1, k2, k3, k4;
    for (std::size_t i = 0; i < u.rows() - 1; i++) {
      const auto uk = u.col(i);
      const auto xk = x_out.col(i);
      sim_step(xk, uk, k1);
      sim_step(xk + k1 * dt_ / 2.0, uk, k2);
      sim_step(xk + k2 * dt_ / 2.0, uk, k3);
      sim_step(xk + k3 * dt_, uk, k4);
      x_out.col(i + 1) = xk + (dt_ / 6.0f) * (k1 + 2.0f * k2 + 2.0f * k3 + k4);
    }
  };


 private:
  inline void sim_step(const VectorState& x0, const VectorControl& u,
                       VectorState& dx) {
    static_cast<Derived*>(this)->sim_step(x0, u, dx);
  };

  const float dt_{0.1};
};

};  // namespace robot_dynamics

#endif  // __ROBOT_DYNAMICS_HPP__