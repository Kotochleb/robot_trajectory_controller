#ifndef __ROBOT_DYNAMICS_HPP__
#define __ROBOT_DYNAMICS_HPP__

#include <Eigen/Core>
#include <Eigen/Dense>

namespace robot_dynamics {

typedef double number_t;

// nst is number of state variables
// nctr is number of control inputs
template <class Derived, int nst, int nctr>
struct RobotDynamics {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr int nst_ext = nst + 1;
  static constexpr int npsi = nst + nctr;
  typedef Eigen::Matrix<number_t, nst_ext, Eigen::Dynamic, Eigen::ColMajor>
      MatrixStateExt;
  typedef Eigen::Matrix<number_t, nctr, Eigen::Dynamic, Eigen::ColMajor>
      MatrixControl;
  typedef Eigen::Matrix<number_t, npsi, Eigen::Dynamic, Eigen::ColMajor>
      MatrixPsi;
  typedef Eigen::Matrix<number_t, nst_ext, 1> VectorStateExt;
  typedef Eigen::Matrix<number_t, nst, 1> VectorState;
  typedef Eigen::Matrix<number_t, nctr, 1> VectorControl;
  typedef Eigen::Matrix<number_t, npsi, 1> VectorPsi;
  typedef Eigen::DiagonalMatrix<number_t, nst_ext> MatrixWeights;
  typedef Eigen::Matrix<number_t, nctr, Eigen::Dynamic, Eigen::ColMajor>
      MatrixGrad;

  RobotDynamics(const number_t dt, const MatrixWeights& W) : dt_(dt), W_(W){};

  MatrixStateExt rk4(const MatrixControl& u) { return rk4(u, x0_, xf_); }

  MatrixStateExt rk4(const MatrixControl& u, const VectorStateExt& x0,
                     const VectorStateExt& xf) {
    MatrixStateExt x_out(x0.rows(), u.cols());
    x_out.col(0) = x0;
    for (Eigen::Index i = 0; i < u.cols() - 1; i++) {
      const auto& uk = u.col(i);
      const auto& xk = x_out.col(i);

      const auto k1 = getDX(xk, xf, uk);
      const auto k2 = getDX(xk + k1 * dt_ / 2.0, xf, uk);
      const auto k3 = getDX(xk + k2 * dt_ / 2.0, xf, uk);
      const auto k4 = getDX(xk + k3 * dt_, xf, uk);

      x_out.col(i + 1) = xk + (dt_ / 6.0) * (k1 + k4 + 2.0 * (k2 + k3));
    }
    return x_out;
  };

  inline number_t getCost(const MatrixControl& u) {
    return getCost(x0_, xf_, u);
  }

  inline number_t getCost(const VectorStateExt& x0, const VectorStateExt& xf,
                          const MatrixControl& u) {
    MatrixStateExt x = rk4(u, x0, xf);
    return costFun(x, xf, u);
  };

  inline MatrixGrad getGradient(const MatrixControl& u) {
    return getGradient(x0_, xf_, u);
  }

  inline MatrixGrad getGradient(const VectorStateExt& x0,
                                const VectorStateExt& xf,
                                const MatrixControl& u) {
    MatrixStateExt x = rk4(u, x0, xf);

    VectorPsi psi_f = VectorPsi::Zero();
    psi_f.segment(0, xf.rows()) = -dqDx(x.col(x.cols() - 1), xf);

    MatrixPsi psi_out = MatrixPsi(psi_f.rows(), u.cols());
    rk4Psi(x, u, psi_f, psi_out);

    MatrixGrad phi_integrated =
        psi_out.block(psi_out.rows() - u.rows(), 0, u.rows(), u.cols());
    MatrixGrad g(u.rows(), u.cols());
    for (Eigen::Index i = 1; i < phi_integrated.cols(); i++) {
      g.col(i) = phi_integrated.col(i - 1) - phi_integrated.col(i);
    }
    return g;
  };

  inline void setupState(const VectorState& x0, const VectorState& xf) {
    x0_.segment(0, nst) = x0;
    xf_.segment(0, nst) = xf;
    x0_[nst_ext - 1] = 0.0;
    xf_[nst_ext - 1] = 0.0;
  }

  inline MatrixControl getInitialValues(const int N) {
    return static_cast<Derived*>(this)->getInitialValues(x0_, xf_, N);
  };

  inline MatrixControl getInitialValues(const VectorStateExt& x,
                                        const VectorStateExt& xf, const int N) {
    return static_cast<Derived*>(this)->getInitialValues(x, xf, N);
  };

  constexpr int getNStateVar() { return nst; }
  constexpr int getNExtStateVar() { return nst_ext; }
  constexpr int getNControlVar() { return nctr; }
  constexpr int getNPsiVar() { return npsi; }

 protected:
  const number_t dt_;
  const MatrixWeights W_;
  VectorStateExt x0_;
  VectorStateExt xf_;

 private:
  void rk4Psi(const MatrixStateExt& x, const MatrixControl& u,
              const VectorPsi& psi_f, MatrixPsi& psi_out) {
    assert(u.cols() == psi_out.cols());
    assert(u.cols() == x.cols());

    psi_out.col(psi_out.cols() - 1) = psi_f;

    for (Eigen::Index k = u.cols() - 1; k > 0; k--) {
      const auto& pk = psi_out.col(k);
      const auto& uk1 = u.col(k - 1);
      const auto& xk = x.col(k);
      const auto& xk1 = x.col(k - 1);
      const auto x05 = (xk + xk1) / 2.0;

      const auto dp1 = -getDPsi(xk, uk1, pk);
      const auto dp2 = -getDPsi(x05, uk1, pk + dp1 * dt_ / 2.0);
      const auto dp3 = -getDPsi(x05, uk1, pk + dp2 * dt_ / 2.0);
      const auto dp4 = -getDPsi(x05, uk1, pk + dp3 * dt_);

      psi_out.col(k - 1) = pk + (dt_ / 6.0) * (dp1 + dp4 + 2.0 * (dp2 + dp3));
    };
  };

  inline number_t q(const VectorStateExt& x, const VectorStateExt& xf) {
    return static_cast<Derived*>(this)->q(x, xf);
  };

  inline VectorStateExt getDX(const VectorStateExt& x, const VectorStateExt& xf,
                              const VectorControl& u) {
    return static_cast<Derived*>(this)->getDX(x, xf, u);
  };

  inline VectorPsi getDPsi(const VectorStateExt& x,
                           [[maybe_unused]] const VectorControl& u,
                           const VectorPsi& p) {
    return static_cast<Derived*>(this)->getDPsi(x, u, p);
  };

  inline number_t costFun(const MatrixStateExt& x_out, const VectorStateExt& xf,
                          const MatrixControl& u) {
    return static_cast<Derived*>(this)->costFun(x_out, xf, u);
  };

  inline VectorStateExt dqDx(const VectorStateExt& x,
                             const VectorStateExt& xf) {
    return static_cast<Derived*>(this)->dqDx(x, xf);
  };
};
};  // namespace robot_dynamics

#endif  // __ROBOT_DYNAMICS_HPP__