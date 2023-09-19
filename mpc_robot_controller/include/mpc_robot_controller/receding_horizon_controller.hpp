#ifndef __RECEDING_HORIZON_CONTROLLER_HPP__
#define __RECEDING_HORIZON_CONTROLLER_HPP__

#include <memory>

#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>
#include <ifopt/test_vars_constr_cost.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav_msgs/msg/path.hpp>

#include <mpc_robot_controller/controller_nlp.hpp>
#include <mpc_robot_controller/diff_drive_dynamics.hpp>
#include <mpc_robot_controller/robot_dynamics.hpp>

#include <ifopt/ipopt_solver.h>
#include <set>

namespace receding_horizon_controller {

using Dynamics = diff_drive_dynamics::DiffDriveDynamics;

// template <typename Dynamics>
class RecidingHorizonController {
 public:
  RecidingHorizonController(const std::shared_ptr<Dynamics>& dynamics,
                            const std::size_t max_horizon, const std::size_t max_iter,
                            const double max_cpu_time);

  void roll(const std::size_t n);
  bool predict(const std::size_t n);
  void generatePath();
  void setControlWeights(const std::vector<double> r_vect);
  void setStateWeights(const std::vector<double> w_vect);
  void clearControlMemory();
  void randomizeControlMemory();
  void setColdStart();
  void setWarmStart();
  void resetOptimuizer();
  void setMap(const robot_dynamics::ReduceMap& rm);
  void setState(const geometry_msgs::msg::Twist& twist_start,
                const geometry_msgs::msg::PoseStamped& pose_end,
                const geometry_msgs::msg::Twist& twist_end);
  void setNIter(const std::size_t max_iter);

  std::vector<geometry_msgs::msg::Twist> getVelocityCommands(const unsigned n);
  nav_msgs::msg::Path getPath(const std::size_t granulaty);

 private:
  std::shared_ptr<Dynamics> dynamics_;
  std::size_t max_horizon_;
  int max_iter_;
  const double max_cpu_time_;

  Dynamics::MatrixState x_;
  Dynamics::MatrixControl u_;

  std::unique_ptr<ifopt::IpoptSolver> ipopt_;
  std::shared_ptr<controller_nlp::ControllerCost> cost_;
  std::shared_ptr<controller_nlp::ControllerConstraint> constraint_;
  std::shared_ptr<controller_nlp::ControllerVariables> variables_;

  const std::set<int> ok_responses_ = {0, 1, 2, 3, 4, 5, -1, -4};
};

};  // namespace receding_horizon_controller

#endif  // __ROBOT_DYNAMICS_HPP__