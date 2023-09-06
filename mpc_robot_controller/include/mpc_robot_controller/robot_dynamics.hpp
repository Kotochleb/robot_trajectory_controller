#ifndef __ROBOT_DYNAMICS_HPP__
#define __ROBOT_DYNAMICS_HPP__

#include <Eigen/Core>
#include <Eigen/Dense>

namespace robot_dynamics {

struct CostMap {
  double resolution;
  unsigned cells_x;
  unsigned cells_y;
  unsigned char thresh;
  unsigned char* map;
};

using MapPoint = std::pair<double, double>;
using ReduceMap = std::vector<MapPoint>;

// nst is number of state variables
// nctr is number of control inputs
template <class Derived, int nst, int nctr>
struct RobotDynamics {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr int nst_ext = nst + 1;
  static constexpr int npsi = nst + nctr;
  typedef Eigen::Matrix<double, nst, Eigen::Dynamic, Eigen::ColMajor> MatrixState;
  typedef Eigen::Matrix<double, nst_ext, Eigen::Dynamic, Eigen::ColMajor> MatrixStateExt;
  typedef Eigen::Matrix<double, nctr, Eigen::Dynamic, Eigen::ColMajor> MatrixControl;
  typedef Eigen::Matrix<double, npsi, Eigen::Dynamic, Eigen::ColMajor> MatrixPsi;
  typedef Eigen::Matrix<double, nst_ext, 1> VectorStateExt;
  typedef Eigen::Matrix<double, nst, 1> VectorState;
  typedef Eigen::Matrix<double, nctr, 1> VectorControl;
  typedef Eigen::Matrix<double, npsi, 1> VectorPsi;
  typedef Eigen::DiagonalMatrix<double, nst> MatrixWeights;
  typedef Eigen::DiagonalMatrix<double, nst_ext> MatrixWeightsExt;
  typedef Eigen::Matrix<double, nctr, Eigen::Dynamic, Eigen::ColMajor> MatrixGrad;

  RobotDynamics(const double dt) : dt_(dt){};

  inline MatrixState rk4(const MatrixControl& u) { return rk4Ext(u, x0_, xf_).topRows(nst); }

  inline double getCost(const MatrixControl& u) { return getCost(x0_, xf_, u); }

  inline double getCost(const VectorStateExt& x0, const VectorStateExt& xf,
                        const MatrixControl& u) {
    MatrixStateExt x = rk4Ext(u, x0, xf);
    return costFun(x, xf, u);
  };

  inline MatrixGrad getGradient(const MatrixControl& u) { return getGradient(x0_, xf_, u); }

  inline MatrixGrad getGradient(const VectorStateExt& x0, const VectorStateExt& xf,
                                const MatrixControl& u) {
    MatrixStateExt x = rk4Ext(u, x0, xf);

    VectorPsi psi_f = VectorPsi::Zero();
    psi_f.head(xf.rows()) = -dqDx(x.col(x.cols() - 1), xf);

    MatrixPsi psi_out = MatrixPsi(psi_f.rows(), u.cols());
    rk4ExtPsi(x, u, psi_f, psi_out);

    MatrixGrad phi_integrated = psi_out.block(psi_out.rows() - u.rows(), 0, u.rows(), u.cols());
    MatrixGrad g(u.rows(), u.cols());

    for (Eigen::Index i = 0; i < phi_integrated.cols(); i++) {
      g.col(i) = -getDPsi(x.col(i), u.col(i), psi_out.col(i)).tail(nctr);
    }
    return g;
  };

  inline void setupState(const VectorState& x0, const VectorState& xf) {
    x0_.head(nst) = x0;
    xf_.head(nst) = xf;
    x0_.tail(1).setZero();
    xf_.tail(1).setZero();
  }

  inline MatrixControl getInitialValues(const int N) {
    return static_cast<Derived*>(this)->getInitialValues(x0_, xf_, N);
  };

  inline MatrixControl getInitialValues(const VectorStateExt& x, const VectorStateExt& xf,
                                        const int N) {
    return static_cast<Derived*>(this)->getInitialValues(x, xf, N);
  };

  inline void setWeightMatrix(const MatrixWeights& W) {
    W_.diagonal().head(nst) = W.diagonal();
    W_.diagonal().tail(1).setZero();
  }

  inline void setSigmaMap(const CostMap& map, const double sigma) {
    static_cast<Derived*>(this)->setSigmaMap(map, sigma);
  }

  inline void generateReducedCostmap(const CostMap& map) {
    static_cast<Derived*>(this)->generateReducedCostmap(map);
  }

  constexpr int getNStateVar() { return nst; }
  constexpr int getNExtStateVar() { return nst_ext; }
  constexpr int getNControlVar() { return nctr; }
  constexpr int getNPsiVar() { return npsi; }

 protected:
  const double dt_;
  MatrixWeightsExt W_;
  VectorStateExt x0_;
  VectorStateExt xf_;

 private:
  inline MatrixStateExt rk4Ext(const MatrixControl& u) { return rk4Ext(u, x0_, xf_); }

  inline MatrixStateExt rk4Ext(const MatrixControl& u, const VectorStateExt& x0,
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

  void rk4ExtPsi(const MatrixStateExt& x, const MatrixControl& u, const VectorPsi& psi_f,
                 MatrixPsi& psi_out) {
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

  inline double q(const VectorStateExt& x, const VectorStateExt& xf, const VectorControl& u) {
    return static_cast<Derived*>(this)->q(x, xf, u);
  };

  inline VectorStateExt getDX(const VectorStateExt& x, const VectorStateExt& xf,
                              const VectorControl& u) {
    return static_cast<Derived*>(this)->getDX(x, xf, u);
  };

  inline VectorPsi getDPsi(const VectorStateExt& x, [[maybe_unused]] const VectorControl& u,
                           const VectorPsi& p) {
    return static_cast<Derived*>(this)->getDPsi(x, u, p);
  };

  inline double costFun(const MatrixStateExt& x_out, const VectorStateExt& xf,
                        const MatrixControl& u) {
    return static_cast<Derived*>(this)->costFun(x_out, xf, u);
  };

  inline VectorStateExt dqDx(const VectorStateExt& x, const VectorStateExt& xf) {
    return static_cast<Derived*>(this)->dqDx(x, xf);
  };
};
};  // namespace robot_dynamics

#endif  // __ROBOT_DYNAMICS_HPP__