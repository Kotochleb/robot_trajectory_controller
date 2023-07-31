#ifndef __DIFF_DRIVE_DYNAMICS_HPP__
#define __DIFF_DRIVE_DYNAMICS_HPP__

// #include "IpTypes.hpp"
#include <cmath>

#include <iostream>

#include <Eigen/Dense>

#include <robot_dynamics.hpp>

namespace diff_drive_dynamics {
using namespace Eigen;

static constexpr std::size_t state_variables = 5;
static constexpr std::size_t control_inputs = 2;

struct DiffDriveDynamics
    : robot_dynamics::RobotDynamics<DiffDriveDynamics, state_variables, control_inputs> {
 public:
  typedef Matrix<float, state_variables, Dynamic> MatrixState;
  typedef Matrix<float, control_inputs, Dynamic> MatrixControl;
  typedef Matrix<float, state_variables, 1> VectorState;
  typedef Matrix<float, control_inputs, 1> VectorControl;
  friend class robot_dynamics::RobotDynamics<DiffDriveDynamics, state_variables, control_inputs>;

  DiffDriveDynamics(const float dt, const float wheel_separation,
                    const float wheel_radius) {};

 private:
  inline void sim_step(const VectorState& x0, const VectorControl& u,
                       VectorState& dx) {
    dx[0] = u[0];
    dx[1] = x0[0] * sin(x0[4]);
    dx[2] = x0[0] * cos(x0[4]);
    dx[3] = u[1];
    dx[4] = x0[3];
  }
};

};  // namespace diff_drive_dynamics

#endif  // __DIFF_DRIVE_DYNAMICS_HPP__