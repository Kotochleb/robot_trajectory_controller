#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>
#include <ifopt/test_vars_constr_cost.h>

#include <diff_drive_dynamics.hpp>
#include <robot_state_plotter.hpp>

#include <controller_nlp.hpp>

#include <Eigen/Dense>

namespace plt = matplot;

int main([[maybe_unused]] int argc, [[maybe_unused]] char* argv[]) {

  auto dt = 0.05;
  const std::size_t N = 1000;

  diff_drive_dynamics::DiffDriveDynamics::MatrixWeights W;
  W.setIdentity();
  auto rd =
      std::make_shared<diff_drive_dynamics::DiffDriveDynamics>(0.0, 0.0, dt, W);

  diff_drive_dynamics::DiffDriveDynamics::VectorState x0;
  x0 << 0.0, 0.0, 0.0, 0.0, 0.0;
  diff_drive_dynamics::DiffDriveDynamics::VectorState xf;
  xf << 0.0, 5.0, 0.0, 0.0, 0.0;
  rd->setupState(x0, xf);

  auto variables = std::make_shared<controller_nlp::ControllerVariables>(rd, N);
  auto constraint = std::make_shared<controller_nlp::ControllerConstraint>(rd);
  auto cost = std::make_shared<controller_nlp::ControllerCost>(rd);

  ifopt::Problem nlp;
  nlp.AddVariableSet(variables);
  nlp.AddConstraintSet(constraint);
  nlp.AddCostSet(cost);
  nlp.PrintCurrent();

  ifopt::IpoptSolver ipopt;
  // ipopt.SetOption("linear_solver", "mumps");
  // ipopt.SetOption("jacobian_approximation", "exact");

  ipopt.Solve(nlp);
  Eigen::VectorXd vals = nlp.GetOptVariables()->GetValues();
  std::cout << vals.transpose() << std::endl;

  diff_drive_dynamics::DiffDriveDynamics::MatrixControl u(2, vals.rows() / 2);
  u.row(0) = vals.segment(0, vals.rows() / 2);
  u.row(1) = vals.segment(vals.rows() / 2, vals.rows() / 2);

  const auto x_out = rd->rk4(u);
  auto rsp = robot_state_plotter::RobotStatePlotter(dt);
  rsp.plot(u, x_out);
}
