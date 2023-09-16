#ifndef __CONTROLLER_NLP_HPP__
#define __CONTROLLER_NLP_HPP__

#include <memory>

#include <Eigen/Dense>

#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <ifopt/variable_set.h>

#include <mpc_robot_controller/diff_drive_dynamics.hpp>

namespace controller_nlp {
using Dynamics = diff_drive_dynamics::DiffDriveDynamics;

class ControllerVariables : public ifopt::VariableSet {
 public:
  ControllerVariables(const std::shared_ptr<Dynamics>& dynamics, const int horizon)
      : ifopt::VariableSet(horizon * ctr_mat_cols, "controller_var_set"),
        dynamics_(dynamics),
        horizon_(horizon) {
    const auto u0 = dynamics_->getInitialValues(horizon);
    state_ = u0.reshaped<Eigen::RowMajor>().transpose();
  }

  void SetVariables(const ifopt::Component::VectorXd& x) override { state_ = x; };

  ifopt::Component::VectorXd GetValues() const override { return state_; };

  ifopt::Component::VecBound GetBounds() const override {
    VecBound bounds(GetRows());
    const double lin_lim = dynamics_->getLinearAccelerationLimit();
    const double ang_lim = dynamics_->getAngularAccelerationLimit();
    const auto bound_lin = ifopt::Bounds(-lin_lim, lin_lim);
    const auto bound_ang = ifopt::Bounds(-ang_lim, ang_lim);
    std::fill_n(bounds.begin(), bounds.size() / 2, bound_lin);
    std::fill_n(bounds.begin() + (bounds.size() / 2), bounds.size() / 2, bound_ang);
    return bounds;
  }

 private:
  std::shared_ptr<Dynamics> dynamics_;
  ifopt::Component::VectorXd state_;
  const int horizon_;
  static constexpr int ctr_mat_cols = 2;
  static constexpr int x_rows = 5;
};

class ControllerConstraint : public ifopt::ConstraintSet {
 public:
  ControllerConstraint(const std::shared_ptr<Dynamics>& dynamics)
      : ifopt::ConstraintSet(0, "controller_constrains"), dynamics_(dynamics) {}

  ifopt::Component::VectorXd GetValues() const override {
    return ifopt::Component::VectorXd(GetRows());
  };

  ifopt::Component::VecBound GetBounds() const override {
    return ifopt::Component::VecBound(GetRows());
  }

  void FillJacobianBlock([[maybe_unused]] std::string var_set,
                         [[maybe_unused]] ifopt::Component::Jacobian& jac_block) const override {}

 private:
  std::shared_ptr<Dynamics> dynamics_;
};

class ControllerCost : public ifopt::CostTerm {
 public:
  ControllerCost(const std::shared_ptr<Dynamics>& dynamics)
      : ifopt::CostTerm("controller_cost"), dynamics_(dynamics) {}

  double GetCost() const override {
    const auto vals = GetVariables()->GetComponent("controller_var_set")->GetValues();
    const auto u = vals.reshaped<Eigen::RowMajor>(2, vals.rows() / 2);
    const double cost = dynamics_->getCost(u);
    return cost;
  };

  void FillJacobianBlock([[maybe_unused]] std::string var_set,
                         [[maybe_unused]] ifopt::Component::Jacobian& jac) const override {
    if (var_set == "controller_var_set") {
      const auto vals = GetVariables()->GetComponent("controller_var_set")->GetValues();
      const auto u = vals.reshaped<Eigen::RowMajor>(2, vals.rows() / 2);

      const auto g = dynamics_->getGradient(u);
      const auto serial_g = g.reshaped<Eigen::RowMajor>().transpose();

      for (Eigen::Index i = 0; i < serial_g.cols(); i++) {
        jac.coeffRef(0, i) = serial_g(0, i);
      }
    }
  }

 private:
  Dynamics::MatrixControl u_;
  std::shared_ptr<Dynamics> dynamics_;
};
};      // namespace controller_nlp
#endif  // __CONTROLLER_NLP_HPP__
