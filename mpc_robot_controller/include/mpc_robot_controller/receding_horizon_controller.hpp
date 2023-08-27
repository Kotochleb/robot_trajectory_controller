#ifndef __RECEDING_HORIZON_CONTROLLER_HPP__
#define __RECEDING_HORIZON_CONTROLLER_HPP__

#include <memory>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav_msgs/msg/path.hpp>

#include <diff_drive_dynamics.hpp>
#include <robot_dynamics.hpp>

namespace receding_horizon_controller {

using Dynamics = diff_drive_dynamics::DiffDriveDynamics;

// template <typename Dynamics>
class RecidingHorizonController {
 public:
  RecidingHorizonController(const std::shared_ptr<Dynamics>& dynamics) : dynamics_(dynamics);
  ~RecidingHorizonController();

  void roll(const std::size_t n);
  void predict(const std::size_t n);
  void generatePath();
  void setChaseWeights();
  void setPositioningWeights();
  void setMap(const nav2_costmap_2d::Costmap2D* map);
  void setFinalState(const geometry_msgs::msg::PoseStamped& pose);
  void setNIter(const std::size_t max_iter);

  nav_msgs::msg::Path getPath(const std::size_t granulaty);
  geometry_msgs::msg::Twist getVelocityCommand();

 private:
  std::size_t max_iter_;
  std::shared_ptr<Dynamics> dynamics_;
  Dynamics::MatrixState x_;
  Dynamics::MatrixControl u_;

  ifopt::Problem nlp_;
  ifopt::IpoptSolver ipopt_;
  std::make_shared<controller_nlp::ControllerConstraint> constraint_;
  std::make_shared<controller_nlp::ControllerCost> cost_;
};

};  // namespace receding_horizon_controller

#endif  // __ROBOT_DYNAMICS_HPP__