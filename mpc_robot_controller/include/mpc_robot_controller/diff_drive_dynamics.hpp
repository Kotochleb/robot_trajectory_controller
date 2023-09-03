#ifndef __DIFF_DRIVE_DYNAMICS_HPP__
#define __DIFF_DRIVE_DYNAMICS_HPP__

#include <cmath>

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
  double acceleration;
  double wheel_separation;
  double wheel_radius;
};

struct CostMap {
  double resolution;
  unsigned cells_x;
  unsigned cells_y;
  unsigned char* map;
};

using robot_dynamics::RobotDynamics;
static constexpr int state_variables = 5;
static constexpr int control_inputs = 2;

struct DiffDriveDynamics : RobotDynamics<DiffDriveDynamics, state_variables, control_inputs> {
 public:
  friend class RobotDynamics<DiffDriveDynamics, state_variables, control_inputs>;
  DiffDriveDynamics(const DiffDriveParams& params)
      : RobotDynamics<DiffDriveDynamics, state_variables, control_inputs>(params.dt),
        parmas_(params){

        };

  inline MatrixControl getInitialValues(const int N) {
    MatrixControl out = MatrixControl::Zero(control_inputs, N);
    return out;
  };

  inline double getVelocityForwardLimit() { return parmas_.velocity.forward; };
  inline double getVelocityBackwardLimit() { return parmas_.velocity.backward; };
  inline double getVelocityAngularLimit() { return parmas_.velocity.angular; };
  inline double getAccelerationLimit() { return parmas_.acceleration; };

  inline void setMap(const CostMap& map) { map_ = map; }

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

  inline VectorPsi getDPsi(const VectorStateExt& x, const VectorControl& /*u*/,
                           const VectorPsi& p) {
    VectorPsi dp;
    dp[0] = -cos(x[4]) * p[1] - sin(x[4]) * p[2];
    dp[1] = 0.0;
    dp[2] = 0.0;
    dp[3] = -p[4];
    dp[4] = x[0] * sin(x[4]) * p[1] - x[0] * cos(x[4]) * p[2];

    dp.segment<5>(0).noalias() += dqDx(x, xf_).segment<5>(0);

    dp[5] = p[0];
    dp[6] = p[3];
    return dp;
  };

  inline double q(const VectorStateExt& x, const VectorStateExt& xf, const VectorControl& u) {

    const VectorStateExt dx_end = dXF(x, xf);
    const double map_cost = getCostmapCost(x, map_, sigma_);
    // return dx_end.transpose() * W_ * dx_end + x[5] + map_cost;
    return 50.0 * (pow(u[0], 2) + pow(u[1], 2)) +
           20.0 * (std::pow(dx_end[1], 2) + std::pow(dx_end[2], 2)) + map_cost;
  }

  inline VectorStateExt dqDx(const VectorStateExt& x, const VectorStateExt& xf) {
    // VectorStateExt dq;
    // dq.noalias() -= xf;
    VectorStateExt dq = dXF(x, xf);
    return W_ * dq;
  }

  inline double costFun(const MatrixStateExt& x_out, const VectorStateExt& xf,
                        const MatrixControl& u) {

    const double target_difference = q(x_out.col(x_out.cols() - 1), xf, u);
    const double state_cost = x_out(x_out.rows() - 1, x_out.cols() - 1);
    return 0.5 * target_difference + state_cost;
  };

  // auxiliary function to computing minimum angle theta
  inline VectorStateExt dXF(const VectorStateExt& x, const VectorStateExt& xf) {
    VectorStateExt dxf = x;
    dxf.head(4).noalias() -= xf.head(4);
    // dxf[4] = atan2(sin(x[4] - xf[4]), cos(x[4] - xf[4]));
    dxf[4] = x[4] - xf[4];
    dxf[5] = 0.0;
    return dxf;
  }

  // compute the cost related to the map
  inline double getCostmapCost([[maybe_unused]] const VectorStateExt& xk,
                               [[maybe_unused]] const CostMap& map,
                               [[maybe_unused]] const double sigma) {
    return 0.0;
    // const double x = xk[1];
    // const double y = xk[2];
    // const double sigma_map_pow = std::pow(map.resolution * sigma, 2.0);
    // const double coef = 1 / std::sqrt(2.0 * M_PI * sigma_map_pow);
    // const double exp_coef = -0.5 / (sigma_map_pow);
    // // handle cases of odd and even number of cells
    // const double grid_offset_x = map.cells_x % 2 ? 0.0 : 0.5;
    // const double grid_offset_y = map.cells_y % 2 ? 0.0 : 0.5;

    // double acc = 0.0;

    // const int mcx = static_cast<int>(map.cells_x);
    // const int mcy = static_cast<int>(map.cells_y);
    // for (int iy = -mcy / 2; iy < mcy / 2; iy++) {
    //   for (int ix = -mcx / 2; ix < mcx; ix++) {
    //     const double map_x_dist = x - (ix + grid_offset_x) * map.resolution;
    //     const double map_y_dist = y - (iy + grid_offset_y) * map.resolution;
    //     const double square_dist = map_x_dist * map_x_dist + map_y_dist * map_y_dist;
    //     const int map_idx = (iy + mcy) * mcx + (ix + mcx);
    //     const double wi = static_cast<double>(map.map[map_idx]) / 100.0;
    //     acc += wi * std::exp(exp_coef * square_dist);
    //   }
    // }
    // return acc * coef;
  }

  const double sigma_ = 5.0;
  const DiffDriveParams parmas_;
  CostMap map_;
};

};  // namespace diff_drive_dynamics

#endif  // __DIFF_DRIVE_DYNAMICS_HPP__