#ifndef __DIFF_DRIVE_DYNAMICS_HPP__
#define __DIFF_DRIVE_DYNAMICS_HPP__

#include <cmath>

#include <Eigen/Dense>

#include <robot_dynamics.hpp>

namespace diff_drive_dynamics {

using robot_dynamics::number_t;
using robot_dynamics::RobotDynamics;
static constexpr int state_variables = 5;
static constexpr int control_inputs = 2;

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

  inline MatrixControl getInitialValues(const int N) {
    MatrixControl out = MatrixControl::Zero(control_inputs, N);
    return out;
  };

 private:
  inline MatrixControl getInitialValues(
      [[maybe_unused]] const VectorStateExt& x,
      [[maybe_unused]] const VectorStateExt& xf, const int N) {
    return MatrixControl::Zero(control_inputs, N);
  };

  inline VectorStateExt getDX(const VectorStateExt& x, const VectorStateExt& xf,
                              const VectorControl& u) {
    VectorStateExt dx;
    dx[0] = u[0];
    dx[1] = x[0] * sin(x[4]);
    dx[2] = x[0] * cos(x[4]);
    dx[3] = u[1];
    dx[4] = x[3];

    const VectorStateExt dxf = dXF(x, xf);
    dx[5] = dxf.array().square().sum();
    return dx;
  }

  inline VectorPsi getDPsi(const VectorStateExt& x,
                           [[maybe_unused]] const VectorControl& u,
                           const VectorPsi& p) {
    VectorPsi dp;
    dp[0] = -cos(x[4]) * p[1] - sin(x[4]) * p[2];
    dp[1] = 0.0;
    dp[2] = 0.0;
    dp[3] = -p[3];
    dp[4] = x[0] * sin(x[4]) * p[1] - x[0] * cos(x[4]) * p[2];
    dp[5] = p[0] * x[0];
    dp[6] = p[3] * x[3];
    return dp;
  };

  inline number_t costFun(const MatrixStateExt& x_out, const VectorStateExt& xf,
                          [[maybe_unused]] const MatrixControl& u) {

    const VectorStateExt dx_end = dXF(x_out.col(x_out.cols() - 1), xf);
    const number_t state_cost = x_out(x_out.rows() - 1, x_out.cols() - 1);
    const number_t target_difference = dx_end.transpose() * W_ * dx_end;
    return state_cost + target_difference / 2.0;
  };

  inline VectorPsi getPsiF(const VectorStateExt& x, const VectorStateExt& xf) {
    const VectorStateExt dx_end = dXF(x, xf);
    VectorPsi psi_f;
    psi_f.segment<dx_end.rows()>(0) = -1.0 * (W_ * dx_end);
    return psi_f;
  };

  // auxiliary function to computing minimum angle theta
  inline VectorStateExt dXF(const VectorStateExt& x, const VectorStateExt& xf) {
    VectorStateExt dxf = x;
    dxf.segment<3>(0).noalias() -= xf.segment<3>(0);
    dxf[4] = atan2(sin(xf[4] - x[4]), cos(xf[4] - x[4]));
    dxf[5] = 0.0;
    return dxf;
  }

  const number_t wheel_separation_;
  const number_t wheel_radius_;
};

};  // namespace diff_drive_dynamics

#endif  // __DIFF_DRIVE_DYNAMICS_HPP__