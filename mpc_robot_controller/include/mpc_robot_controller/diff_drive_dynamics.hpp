#ifndef __DIFF_DRIVE_DYNAMICS_HPP__
#define __DIFF_DRIVE_DYNAMICS_HPP__

#include <array>
#include <cmath>
#include <numeric>
#include <vector>

#include <Eigen/Dense>

#include <mpc_robot_controller/robot_dynamics.hpp>

namespace diff_drive_dynamics {

struct DiffDriveParams {
  double dt;
  struct {
    double forward;
    double backward;
    double angular;
  } velocity;
  struct {
    double linear;
    double angular;
  } acceleration;
  double wheel_separation;
  double wheel_radius;
  double max_joint_speed;
};

struct MapConstants {
  float coef;
  float exp_coef;
  float inscribed_radious;
};

using robot_dynamics::RobotDynamics;
static constexpr int state_variables = 5;
static constexpr int control_inputs = 2;

struct DiffDriveDynamics : RobotDynamics<DiffDriveDynamics, state_variables, control_inputs> {
 public:
  friend class RobotDynamics<DiffDriveDynamics, state_variables, control_inputs>;
  DiffDriveDynamics(const DiffDriveParams& params)
      : RobotDynamics<DiffDriveDynamics, state_variables, control_inputs>(params.dt),
        half_width_(params.wheel_separation * 0.5),
        d_wheel_(params.wheel_radius * 2.0),
        parmas_(params),
        rm_(0){

        };

  inline MatrixControl getInitialValues(const int N) {
    MatrixControl out = MatrixControl::Zero(control_inputs, N);
    return out;
  };

  inline void setSigmaMap(const robot_dynamics::ReduceMap& rm, const double resolution,
                          const double sigma, const double inscribed_radious) {
    const double sigma_map_pow = std::pow(resolution * sigma, 2.0);
    map_constants_.coef = 1.0 / std::sqrt(2.0 * M_PI * sigma_map_pow);
    map_constants_.exp_coef = -0.5 / (sigma_map_pow);
    map_constants_.inscribed_radious = inscribed_radious;
    rm_ = rm;
  }

  // compute the cost related to the map
  inline double getCostmapCost(const VectorStateExt& xk, const robot_dynamics::ReduceMap& rm,
                               const MapConstants& constants) {
    const auto cost = [xk, constants](const float s, const robot_dynamics::MapPoint& p) {
      const float x = xk[1] - p.first;
      const float y = xk[2] - p.second;
      const float dist = std::hypot(x, y)  - constants.inscribed_radious;

      const float e = std::exp(-20.0 * dist);
      const float inscribed = 200.0 / (1.0 + e);
      // const float inflation = 10.0 * e;
      return s + inscribed;
    };
    return std::accumulate(rm.begin(), rm.end(), 0.0, cost);
  }

  // compute the cost related to the map
  inline VectorStateExt getCostmapDx(const VectorStateExt& xk, const robot_dynamics::ReduceMap& rm,
                                     const MapConstants& constants) {

    VectorStateExt dp;
    dp.setZero();
    for (const auto& p : rm) {
      const float x = xk[1] - p.first;
      const float y = xk[2] - p.second;
      const float v_dist = std::hypot(x, y);
      const float dist = v_dist - constants.inscribed_radious;

      // const float a = 10.0;
      const float e = std::exp(-10.0 * dist);
      const float inscribed_grad = 200.0 * 10.0 * e / std::pow(1.0 + e, 2.0);
      // const float inflation_grad = -10.0 * std::exp(-dist);

      dp[1] += -x / v_dist * (inscribed_grad);
      dp[2] += -y / v_dist * (inscribed_grad);
    }

    return dp;
  }

  inline double getVelocityForwardLimit() { return parmas_.velocity.forward; };
  inline double getVelocityBackwardLimit() { return parmas_.velocity.backward; };
  inline double getVelocityAngularLimit() { return parmas_.velocity.angular; };
  inline double getLinearAccelerationLimit() { return parmas_.acceleration.linear; };
  inline double getAngularAccelerationLimit() { return parmas_.acceleration.angular; };
  inline robot_dynamics::ReduceMap getReducedMap() { return rm_; };
  inline MapConstants getMapConstants() { return map_constants_; };

 private:
  inline MatrixControl getInitialValues(const VectorStateExt& /*x*/, const VectorStateExt& /*xf*/,
                                        const int N) {
    return MatrixControl::Zero(control_inputs, N);
  };

  inline VectorStateExt getDX(const VectorStateExt& x, const VectorStateExt& xf,
                              const VectorControl& u) {
    VectorStateExt dx;
    dx[0] = u[0];
    dx[1] = x[0] * cos(x[4]);
    dx[2] = x[0] * sin(x[4]);
    dx[3] = u[1];
    dx[4] = x[3];
    dx[5] = q(x, xf, u);
    return dx;
  }

  inline VectorPsi getDPsi(const VectorStateExt& x, const VectorControl& u, const VectorPsi& p) {
    VectorPsi dp;
    dp[0] = -(p[1] * cos(x[4]) + p[2] * sin(x[4]));
    dp[1] = 0.0;
    dp[2] = 0.0;
    dp[3] = -p[4];
    dp[4] = -x[0] * (-p[1] * sin(x[4]) + p[2] * cos(x[4]));

    dp.head(5).noalias() += dqDx(x, xf_).head(5);

    dp[5] = p[0];
    dp[6] = p[3];

    dp.tail(2).noalias() += dqDu(x, xf_, u);

    return dp;
  };

  inline double q(const VectorStateExt& x, const VectorStateExt& xf, const VectorControl& u) {

    const VectorStateExt dx_end = dXF(x, xf);
    const double state_cost = dx_end.transpose() * W_ * dx_end;
    const double control_cost = u.transpose() * R_ * u;

    const double vel_lim_cost = -10.0 * x[0] * x[0] + 2.0 * parmas_.velocity.forward;
    const double rot_lim_cost = -10.0 * x[3] * x[3] + 2.0 * parmas_.velocity.angular;

    // do not compute if empty array
    const double map_cost = rm_.size() ? getCostmapCost(x, rm_, map_constants_) : 0.0;
    return 0.5 * (state_cost + control_cost) + map_cost + vel_lim_cost + rot_lim_cost;
  }

  inline VectorStateExt dqDx(const VectorStateExt& x, const VectorStateExt& xf) {
    const VectorStateExt dq = dXF(x, xf);
    const VectorStateExt map_deriv = getCostmapDx(x, rm_, map_constants_);
    VectorStateExt ext_cost;
    ext_cost.setZero();
    ext_cost[0] = 2.0 * 10.0 * x[0];
    ext_cost[3] = 2.0 * 10.0 * x[3];
    return W_ * dq + map_deriv + ext_cost;
  }

  inline VectorControl dqDu(const VectorStateExt& /*x*/, const VectorStateExt& /*xf*/,
                            const VectorControl& u) {
    const VectorControl dq = u;
    return R_ * dq;
  }

  inline double costFun(const MatrixStateExt& x_out, const VectorStateExt& xf,
                        const MatrixControl& u) {

    const double target_difference = q(x_out.rightCols<1>(), xf, u);
    const double state_cost = x_out.bottomRightCorner<1, 1>()(0, 0);
    return 0.5 * target_difference + state_cost;
  };

  // auxiliary function to computing minimum angle theta
  inline VectorStateExt dXF(const VectorStateExt& x, const VectorStateExt& xf) {
    VectorStateExt dxf = x;
    dxf.head(5).noalias() -= xf.head(5);
    dxf[5] = 0.0;
    return dxf;
  }

  const double half_width_;
  const double d_wheel_;
  MapConstants map_constants_;
  const DiffDriveParams parmas_;
  robot_dynamics::ReduceMap rm_;
};

};  // namespace diff_drive_dynamics

#endif  // __DIFF_DRIVE_DYNAMICS_HPP__