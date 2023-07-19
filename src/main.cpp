#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"
#include "pendulum.hpp"

#include <sciplot/sciplot.hpp>

#include <cmath>
#include <iostream>

int main(int argc, char* argv[]) {
  Ipopt::SmartPtr<Ipopt::TNLP> inverted_pendulum = new InvertedPendulum();
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
  auto status = app->Initialize();

  if (status != Ipopt::Solve_Succeeded) {
    std::cout << std::endl
              << std::endl
              << "*** Error during initialization!" << std::endl;
    return static_cast<int>(status);
  }

  status = app->OptimizeTNLP(inverted_pendulum);

  if (status == Ipopt::Solve_Succeeded) {
    Ipopt::Index iter_count = app->Statistics()->IterationCount();
    std::cout << std::endl
              << std::endl
              << "*** The problem solved in " << iter_count << " iterations!"
              << std::endl;

    Ipopt::Number final_obj = app->Statistics()->FinalObjective();
    std::cout << std::endl
              << std::endl
              << "*** The final value of the objective function is "
              << final_obj << '.' << std::endl;
  }

  // sciplot::Plot2D plot;
  // plot.xlabel("x");

  // sciplot::Vec x = sciplot::linspace(0.0, M_PI, 200);
  // plot.drawCurve(x, std::sin(1.0 * x)).label("sin(x)");

  // sciplot::Figure fig = {{plot}};
  // sciplot::Canvas canvas = {{fig}};
  // canvas.show();

  return static_cast<int>(status);
}
