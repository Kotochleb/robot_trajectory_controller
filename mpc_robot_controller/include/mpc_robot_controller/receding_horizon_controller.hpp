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

namespace receding_horizon_controller {

using Dynamics = diff_drive_dynamics::DiffDriveDynamics;

// template <typename Dynamics>
class RecidingHorizonController {
 public:
  RecidingHorizonController(const std::shared_ptr<Dynamics>& dynamics,
                            const std::size_t max_horizon, const std::size_t max_iter,
                            const double max_cpu_time);

  void roll(const std::size_t n);
  void predict(const std::size_t n);
  void generatePath();
  void setChaseWeights();
  void setPositioningWeights();
  void setMap(const nav2_costmap_2d::Costmap2D* map);
  void setState(const geometry_msgs::msg::Twist& twist_start,
                const geometry_msgs::msg::PoseStamped& pose_end,
                const geometry_msgs::msg::Twist& twist_end);
  void setNIter(const std::size_t max_iter);

  geometry_msgs::msg::Twist getVelocityCommand();
  nav_msgs::msg::Path getPath(const std::size_t granulaty);

 private:
  std::shared_ptr<Dynamics> dynamics_;
  std::size_t max_horizon_;
  std::size_t max_iter_;

  Dynamics::MatrixState x_;
  Dynamics::MatrixControl u_;

  ifopt::IpoptSolver ipopt_;
  std::shared_ptr<controller_nlp::ControllerCost> cost_;
  std::shared_ptr<controller_nlp::ControllerConstraint> constraint_;
  std::shared_ptr<controller_nlp::ControllerVariables> variables_;
};

};  // namespace receding_horizon_controller

#endif  // __ROBOT_DYNAMICS_HPP__