// #include "IpIpoptApplication.hpp"
// #include "IpSolveStatistics.hpp"
// #include "robot_tnlp.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include <diff_drive_dynamics.hpp>
#include <robot_state_plotter.hpp>

#include <Eigen/Dense>

namespace plt = matplot;

int main([[maybe_unused]] int argc, [[maybe_unused]] char* argv[]) {
  // Ipopt::SmartPtr<Ipopt::TNLP> inverted_pendulum = new RobotTNLP();
  // Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
  // auto status = app->Initialize();

  // if (status != Ipopt::Solve_Succeeded) {
  //   std::cout << std::endl
  //             << std::endl
  //             << "*** Error during initialization!" << std::endl;
  //   return static_cast<int>(status);
  // }

  // status = app->OptimizeTNLP(inverted_pendulum);

  // if (status == Ipopt::Solve_Succeeded) {
  //   Ipopt::Index iter_count = app->Statistics()->IterationCount();
  //   std::cout << std::endl
  //             << std::endl
  //             << "*** The problem solved in " << iter_count << " iterations!"
  //             << std::endl;

  //   Ipopt::Number final_obj = app->Statistics()->FinalObjective();
  //   std::cout << std::endl
  //             << std::endl
  //             << "*** The final value of the objective function is "
  //             << final_obj << '.' << std::endl;
  // }

  // return static_cast<int>(status);

  auto dt = 0.1f;
  const std::size_t N = 10000;
  diff_drive_dynamics::DiffDriveDynamics::MatrixWeights W;
  W.setIdentity();
  auto rd = diff_drive_dynamics::DiffDriveDynamics(0.0, 0.0, dt, W);

  diff_drive_dynamics::DiffDriveDynamics::MatrixStateExt x_out =
      diff_drive_dynamics::DiffDriveDynamics::MatrixStateExt::Zero(6, N);
  diff_drive_dynamics::DiffDriveDynamics::MatrixControl u =
      diff_drive_dynamics::DiffDriveDynamics::MatrixControl::Zero(2, N);
  diff_drive_dynamics::DiffDriveDynamics::VectorStateExt x0 =
      diff_drive_dynamics::DiffDriveDynamics::VectorStateExt::Zero();
  diff_drive_dynamics::DiffDriveDynamics::VectorStateExt xf =
      diff_drive_dynamics::DiffDriveDynamics::VectorStateExt::Zero();

  u.block<2, 10>(0, 0).setConstant(1.0f);

  rd.rk4(u, x0, xf, x_out);

  robot_dynamics::number_t p;
  diff_drive_dynamics::DiffDriveDynamics::VectorGrad g =
      diff_drive_dynamics::DiffDriveDynamics::VectorGrad::Zero(N);

  rd.get_gradient(x0, xf, u, p, g);
  std::cout << p << std::endl;

  // std::cout << p << std::endl;

  // auto rsp = robot_state_plotter::RobotStatePlotter(0.1);

  //   rsp.print_matrix(u);
  // rsp.plot(u, x_out);
}
