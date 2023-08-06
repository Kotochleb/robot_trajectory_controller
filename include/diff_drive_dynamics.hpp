#ifndef __DIFF_DRIVE_DYNAMICS_HPP__
#define __DIFF_DRIVE_DYNAMICS_HPP__

#include <cmath>

#include <Eigen/Dense>

#include <robot_dynamics.hpp>

namespace diff_drive_dynamics {

using robot_dynamics::number_t;
using robot_dynamics::RobotDynamics;
static constexpr std::size_t state_variables = 5;
static constexpr std::size_t control_inputs = 2;

struct DiffDriveDynamics
    : RobotDynamics<DiffDriveDynamics, state_variables, control_inputs> {
 public:
  friend class RobotDynamics<DiffDriveDynamics, state_variables,
                             control_inputs>;
  DiffDriveDynamics(const number_t wheel_separation,
                    const number_t wheel_radius, const number_t dt,
                    const MatrixWeights& W)
      : RobotDynamics<DiffDriveDynamics, state_variables, control_inputs>(dt,
                                                                          W),
        wheel_separation_(wheel_separation),
        wheel_radius_(wheel_radius){

        };

 private:
  inline void get_d_x(const VectorStateExt& x0, const VectorStateExt& xf,
                      const VectorControl& u, VectorStateExt& dx) {
    dx[0] = u[0];
    dx[1] = x0[0] * sin(x0[4]);
    dx[2] = x0[0] * cos(x0[4]);
    dx[3] = u[1];
    dx[4] = x0[3];

    const VectorStateExt dxf = x0 - xf;
    const RowVectorStateExt dxfT = dxf.transpose();
    const number_t cost = dxfT * dxf;
    dx[5] = cost / 2.0f;
  }

  inline void get_d_psi(const VectorStateExt& x,
                        [[maybe_unused]] const VectorControl& u,
                        const VectorPsi& p, VectorPsi& dp) {
    dp[0] = -cos(x[4]) * p[1] - sin(x[4]) * p[2];
    dp[1] = 0.0f;
    dp[2] = 0.0f;
    dp[3] = -p[3];
    dp[4] = x[0] * sin(x[4]) * p[1] - x[0] * cos(x[4]) * p[2];
    dp[5] = p[0] * x[0] + p[3] * x[3];
  };

  inline number_t cost_fun(const MatrixStateExt& x_out,
                           const VectorStateExt& xf,
                           [[maybe_unused]] const MatrixControl& u) {
    const auto dx_end = x_out.col(x_out.cols() - 1) - xf;
    const number_t state_cost = x_out(x_out.rows() - 1, x_out.cols() - 1);
    const number_t target_difference = dx_end.transpose() * W_ * dx_end;
    return state_cost + target_difference / 2.0f;
  };

  const number_t wheel_separation_;
  const number_t wheel_radius_;
};

};  // namespace diff_drive_dynamics

#endif  // __DIFF_DRIVE_DYNAMICS_HPP__