#ifndef __ROBOT_DYNAMICS_HPP__
#define __ROBOT_DYNAMICS_HPP__

#include <Eigen/Dense>

#include "IpTypes.hpp"

namespace robot_dynamics {

typedef Ipopt::Number number_t;

// nst is number of state variables
// nctr is number of control inputs
template <class Derived, std::size_t nst, std::size_t nctr>
struct RobotDynamics {
 public:
  static constexpr std::size_t nst_ext = nst + 1;
  typedef Eigen::Matrix<number_t, nst_ext, Eigen::Dynamic> MatrixStateExt;
  typedef Eigen::Matrix<number_t, nctr, Eigen::Dynamic> MatrixControl;
  typedef Eigen::Matrix<number_t, nst_ext, Eigen::Dynamic> MatrixPsi;
  typedef Eigen::Matrix<number_t, nst_ext, 1> VectorStateExt;
  typedef Eigen::Matrix<number_t, 1, nst_ext> RowVectorStateExt;
  typedef Eigen::Matrix<number_t, nctr, 1> VectorControl;
  typedef Eigen::Matrix<number_t, nst_ext, 1> VectorPsi;
  typedef Eigen::DiagonalMatrix<number_t, nst_ext> MatrixWeights;
  typedef Eigen::Matrix<number_t, 1, Eigen::Dynamic> VectorGrad;

  RobotDynamics(const number_t dt, const MatrixWeights& W) : dt_(dt), W_(W){};

  void rk4(const MatrixControl& u, const VectorStateExt& x0,
           const VectorStateExt& xf, MatrixStateExt& x_out) {
    assert(u.cols() == x_out.cols());
    x_out.col(0) = x0;
    VectorStateExt k1, k2, k3, k4;
    for (Eigen::Index i = 0; i < u.cols() - 1; i++) {
      const auto& uk = u.col(i);
      const auto& xk = x_out.col(i);
      get_d_x(xk, xf, uk, k1);
      get_d_x(xk + k1 * dt_ / 2.0, xf, uk, k2);
      get_d_x(xk + k2 * dt_ / 2.0, xf, uk, k3);
      get_d_x(xk + k3 * dt_, xf, uk, k4);

      x_out.col(i + 1) = xk;
      x_out.col(i + 1).noalias() += (dt_ / 6.0f) * (k1 + k4 + 2.0f * (k2 + k3));
    }
  };

  void get_gradient(const VectorStateExt& x0, const VectorStateExt& xf,
                    const MatrixControl& u, number_t& p, VectorGrad& g) {
    assert(u.cols() == g.cols());

    MatrixStateExt x = MatrixStateExt(nst_ext, u.cols());
    rk4(u, x0, xf, x);
    p = cost_fun(x, xf, u);

    const VectorPsi psi_f = -1.0f * (W_ * (x.col(x.cols() - 1) - xf));

    MatrixPsi psi_out = MatrixPsi(nst_ext, u.cols());
    rk4_psi(x, u, psi_f, psi_out);
    
    VectorGrad phi_integrated = psi_out.row(psi_out.rows() - 1);
    for (Eigen::Index i = 0; i < phi_integrated.cols() - 1; i++) {
      g[i] = phi_integrated[i] - phi_integrated[i + 1];
    }
  };

 protected:
  const number_t dt_;
  const MatrixWeights W_;

 private:
  void rk4_psi(const MatrixStateExt& x, const MatrixControl& u,
               const VectorPsi& psi_f, MatrixPsi& psi_out) {
    assert(u.cols() == psi_out.cols());
    assert(u.cols() == x.cols());

    psi_out.col(psi_out.cols() - 1) = psi_f;

    VectorPsi dp1, dp2, dp3, dp4;
    for (Eigen::Index k = u.cols() - 1; k > 0; k--) {
      const auto& pk = psi_out.col(k);
      const auto& uk1 = u.col(k - 1);
      const auto& xk = x.col(k);
      const auto& xk1 = x.col(k - 1);
      const auto x05 = (xk + xk1) / 2.0f;

      get_neg_d_psi(xk, uk1, pk, dp1);
      get_neg_d_psi(x05, uk1, pk + dp1 * dt_ / 2.0f, dp2);
      get_neg_d_psi(x05, uk1, pk + dp2 * dt_ / 2.0f, dp3);
      get_neg_d_psi(x05, uk1, pk + dp3 * dt_, dp4);

      psi_out.col(k - 1) = pk;
      psi_out.col(k - 1).noalias() +=
          (dt_ / 6.0f) * (dp1 + dp4 + 2.0f * (dp2 + dp3));
    };
  };
  inline void get_d_x(const VectorStateExt& x0, const VectorStateExt& xf,
                      const VectorControl& u, VectorStateExt& dx) {
    static_cast<Derived*>(this)->get_d_x(x0, xf, u, dx);
  };

  inline void get_neg_d_psi(const VectorStateExt& x, const VectorControl& u,
                            const VectorPsi& p, VectorPsi& dp) {
    get_d_psi(x, u, p, dp);
    dp.array() *= -1.0f;
  };

  inline void get_d_psi(const VectorStateExt& x, const VectorControl& u,
                        const VectorPsi& p, VectorPsi& dp) {
    static_cast<Derived*>(this)->get_d_psi(x, u, p, dp);
  };

  inline number_t cost_fun(const MatrixStateExt& x_out,
                           const VectorStateExt& xf, const MatrixControl& u) {
    return static_cast<Derived*>(this)->cost_fun(x_out, xf, u);
  };
};
};  // namespace robot_dynamics

#endif  // __ROBOT_DYNAMICS_HPP__