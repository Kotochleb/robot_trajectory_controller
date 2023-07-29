#ifndef __ROBOT_DYNAMICS_HPP__
#define __ROBOT_DYNAMICS_HPP__

// #include "IpTypes.hpp"
#include <cmath>

#include <Eigen/Dense>

namespace robot_dynamics {
using namespace Eigen;

template <std::size_t N>
using MatrixState = Matrix<float, 5, N>;

template <std::size_t N>
using MatrixControl = Matrix<float, 2, N>;

typedef Matrix<float, 5, 1> Vector5f;

class RobotDynamics {
 public:
  RobotDynamics(const float wheel_separation, const float wheel_radius){};

  template <std::size_t N>
  void simulate(const MatrixControl<N>& u, const Vector5f& x0,
                MatrixState<N>& x_out) {
    x_out.col(0) = x0;
    Vector5f k1, k2, k3, k4;
    for (std::size_t i = 0; i < N - 1; i++) {
      const auto uk = u.col(i);
      sim_step(x0, uk, k1);
      sim_step(x0 + k1 * dt_ / 2.0, uk, k2);
      sim_step(x0 + k2 * dt_ / 2.0, uk, k3);
      sim_step(x0 + k3 * dt_, uk, k4);
      x_out.col(i + 1) =
          x_out.col(i) + (dt_ / 6.0f) * (k1 + 2.0f * k2 + 2.0f * k3 + k4);
    }
  }

 private:
  inline void sim_step(const Vector5f& x0, const Vector2f& u, Vector5f& xk) {
    xk[0] = x0[0] + u[0];
    xk[1] = x0[1] + x0[0] * sin(x0[4]);
    xk[2] = x0[2] + x0[0] * cos(x0[4]);
    xk[3] = x0[3] + u[1];
    xk[4] = x0[4] + x0[3];
  }
  const float dt_{0.1f};
};

};  // namespace robot_dynamics

#endif  // __ROBOT_DYNAMICS_HPP__