#ifndef __ROBOT_STATE_PLOTTER_HPP__
#define __ROBOT_STATE_PLOTTER_HPP__

#include <iostream>
#include <vector>

#include <matplot/matplot.h>
#include <Eigen/Dense>

#include <diff_drive_dynamics.hpp>

namespace robot_state_plotter {
namespace plt = matplot;
using namespace Eigen;

using robot_dynamics::number_t;

class RobotStatePlotter {
 public:
  RobotStatePlotter(const number_t dt) : dt_(dt){};

  // template <Derived>
  // void print_matrix(const MatrixBase<Derived>& mat) {
  //   std::cout << mat.format(Eigen::IOFormat(4, 0, ", ", "\n", "[", "]"));
  // };

  void plot(const diff_drive_dynamics::DiffDriveDynamics::MatrixControl& u,
            const diff_drive_dynamics::DiffDriveDynamics::MatrixStateExt& x_out) {
    assert(u.cols() == x_out.cols());
    std::vector<number_t> t(u.cols());
    std::iota(t.begin(), t.end(), 0);
    std::for_each(t.begin(), t.end(), [&](number_t& el) { el *= dt_; });

    const std::vector<std::string> y_axis = {"Lin acc", "Ang acc", "Lin vel",
                                             "Ang vel", "Pos y"};
    const std::size_t num_plots = 5;
    std::size_t plt_cnt = 0;

    for (const auto row : u.rowwise()) {
      const auto x = std::vector<number_t>(row.begin(), row.end());
      plt::subplot(num_plots, 1, plt_cnt);
      plt::ylabel(y_axis[plt_cnt]);
      plt::plot(t, x);
      plt_cnt++;
    }

    std::vector<std::size_t> plots = {0, 3};
    for (const auto i : plots) {
      const auto& row = x_out.row(i);
      const auto x = std::vector<number_t>(row.begin(), row.end());
      plt::subplot(num_plots, 1, plt_cnt);
      plt::ylabel(y_axis[plt_cnt]);
      plt::plot(t, x);
      plt_cnt++;
    }

    plt::subplot(num_plots, 1, plt_cnt);
    const auto x = std::vector<number_t>(x_out.row(1).begin(), x_out.row(1).end());
    const auto y = std::vector<number_t>(x_out.row(2).begin(), x_out.row(2).end());
    plt::ylabel(y_axis[plt_cnt]);
    plt::plot(x, y);

    auto f = plt::gcf();
    f->width(f->width() * 2);
    f->height(f->height() * 2);

    plt::show();
  };

 private:
  const number_t dt_;
};

};  // namespace robot_state_plotter

#endif  // __ROBOT_STATE_PLOTTER_HPP__