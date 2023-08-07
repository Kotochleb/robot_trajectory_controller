#ifndef __CONTROLLER_NLP_HPP__
#define __CONTROLLER_NLP_HPP__

#include <memory>

#include <Eigen/Dense>

#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <ifopt/variable_set.h>

#include <diff_drive_dynamics.hpp>
#include <robot_state_plotter.hpp>

namespace controller_nlp {
using Dynamics = diff_drive_dynamics::DiffDriveDynamics;

class ControllerVariables : public ifopt::VariableSet {
 public:
  ControllerVariables(const std::shared_ptr<Dynamics>& dynamics,
                      const int horizon)
      : ifopt::VariableSet(horizon * ctr_mat_cols, "controller_var_set"),
        dynamics_(dynamics),
        horizon_(horizon) {
    ctr_mat_ = dynamics_->getInitialValues(horizon);
  }

  void SetVariables(const ifopt::Component::VectorXd& x) override {
    assert(x.rows() == ctr_mat_cols * horizon_);
    ctr_mat_.row(0) = x.segment(0, horizon_);
    ctr_mat_.row(1) = x.segment(horizon_, horizon_);
  };

  ifopt::Component::VectorXd GetValues() const override {
    ifopt::Component::VectorXd values(ctr_mat_cols * horizon_);
    values.segment(0, horizon_) = ctr_mat_.row(0);
    values.segment(horizon_, horizon_) = ctr_mat_.row(1);
    return values;
  };

  ifopt::Component::VecBound GetBounds() const override {
    VecBound bounds(GetRows());
    const auto bound = ifopt::Bounds(-0.3, 0.3);
    bounds.assign(bounds.size(), bound);
    return bounds;
  }

 private:
  std::shared_ptr<Dynamics> dynamics_;
  Dynamics::MatrixControl ctr_mat_;
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

  void FillJacobianBlock(
      [[maybe_unused]] std::string var_set,
      [[maybe_unused]] ifopt::Component::Jacobian& jac_block) const override {}

 private:
  std::shared_ptr<Dynamics> dynamics_;
};

class ControllerCost : public ifopt::CostTerm {
 public:
  ControllerCost(const std::shared_ptr<Dynamics>& dynamics)
      : ifopt::CostTerm("controller_cost"), dynamics_(dynamics) {}

  double GetCost() const override {
    const auto vals =
        GetVariables()->GetComponent("controller_var_set")->GetValues();
    Dynamics::MatrixControl u(2, vals.rows() / 2);
    u.row(0) = vals.segment(0, vals.rows() / 2);
    u.row(1) = vals.segment(vals.rows() / 2, vals.rows() / 2);
    return dynamics_->getCost(u);
  };

  void FillJacobianBlock(std::string var_set,
                         ifopt::Component::Jacobian& jac) const override {
    if (var_set == "controller_var_set") {
      const auto vals =
          GetVariables()->GetComponent("controller_var_set")->GetValues();
      Dynamics::MatrixControl u(2, vals.rows() / 2);
      u.row(0) = vals.segment(0, vals.rows() / 2);
      u.row(1) = vals.segment(vals.rows() / 2, vals.rows() / 2);
      const auto g = dynamics_->getGradient(u);
      for (Eigen::Index i = 0; i < g.cols(); i++) {
        jac.coeffRef(0, i) = g(0, i);
        jac.coeffRef(0, g.cols() + i) = g(1, i);
      }
    }
  }

 private:
  std::shared_ptr<Dynamics> dynamics_;
};
};      // namespace controller_nlp
#endif  // __CONTROLLER_NLP_HPP__
