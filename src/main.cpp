// #include "IpIpoptApplication.hpp"
// #include "IpSolveStatistics.hpp"
// #include "robot_tnlp.hpp"

#include <cmath>
#include <iostream>
#include <vector>

#include <robot_dynamics.hpp>
#include <robot_state_plotter.hpp>

#include <Eigen/Dense>

namespace plt = matplot;

int main(int argc, char* argv[]) {
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

  auto rd = robot_dynamics::RobotDynamics(1.0f, 1.0f);

  const size_t N = 100;
  robot_dynamics::MatrixState<N> x_out = robot_dynamics::MatrixState<N>::Zero();
  robot_dynamics::MatrixControl<N> u =
      robot_dynamics::MatrixControl<N>::Zero();
  robot_dynamics::Vector5f x0 = robot_dynamics::Vector5f::Zero();

  u.block<2, 10>(0, 0).setConstant(1.0f);

  rd.simulate<N>(u, x0, x_out);

  auto rsp = robot_state_plotter::RobotStatePlotter(0.1);

  // rsp.print_matrix(u);
  rsp.plot<N>(u, x_out);

}
