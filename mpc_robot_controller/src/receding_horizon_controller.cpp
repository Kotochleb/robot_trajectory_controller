#include <mpc_robot_controller/receding_horizon_controller.hpp>

#include <memory>

#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav_msgs/msg/path.hpp>

#include <mpc_robot_controller/controller_nlp.hpp>
#include <mpc_robot_controller/diff_drive_dynamics.hpp>
#include <mpc_robot_controller/robot_dynamics.hpp>

namespace receding_horizon_controller {

RecidingHorizonController::RecidingHorizonController(const std::shared_ptr<Dynamics>& dynamics,
                                                     const std::size_t max_horizon,
                                                     const std::size_t max_iter,
                                                     const double max_cpu_time)
    : dynamics_(dynamics),
      max_horizon_(max_horizon),
      max_iter_(max_iter),
      u_(Dynamics::MatrixControl(Dynamics::MatrixControl::RowsAtCompileTime, max_horizon)) {

  constraint_ = std::make_shared<controller_nlp::ControllerConstraint>(dynamics_);
  cost_ = std::make_shared<controller_nlp::ControllerCost>(dynamics_);

  ipopt_.SetOption("max_iter", int(max_iter_));
  ipopt_.SetOption("print_level", 0);
  ipopt_.SetOption("max_cpu_time", max_cpu_time);
  ipopt_.SetOption("warm_start_init_point", "yes");
  // ipopt_.SetOption("nlp_scaling_method", "gradient-based");
};

void RecidingHorizonController::setMap([[maybe_unused]] const nav2_costmap_2d::Costmap2D* costmap) {
  robot_dynamics::CostMap robot_map;
  robot_map.thresh = 254;
  robot_map.resolution = costmap->getResolution();
  robot_map.cells_x = costmap->getSizeInCellsX();
  robot_map.cells_y = costmap->getSizeInCellsY();
  robot_map.map = costmap->getCharMap();

  dynamics_->setSigmaMap(robot_map, 1.5);
  dynamics_->generateReducedCostmap(robot_map);
}

void RecidingHorizonController::roll(const std::size_t n) {
  const Eigen::Index c = u_.cols();
  if (c <= 0) {
    throw std::runtime_error("Can not roll empty control input!");
  }

  if (n >= std::size_t(c)) {
    u_.leftCols(c - 1) = u_.middleCols(1, c - 2);
    u_.col(c - 1) = u_.col(c - 2);
  } else {
    u_.leftCols(n) = u_.middleCols(1, n);
    u_.rightCols(c - n).setConstant(0.0);
  }
};

void RecidingHorizonController::predict(const std::size_t n) {
  ifopt::Problem nlp_;
  nlp_.AddConstraintSet(constraint_);
  nlp_.AddCostSet(cost_);
  auto variables = std::make_shared<controller_nlp::ControllerVariables>(dynamics_, n);

  // warm start
  auto u_serial = ifopt::Component::VectorXd(u_.rows() * n);
  u_serial.head(n / u_.rows()) = u_.row(0);
  u_serial.tail(n / u_.rows()) = u_.row(1);
  variables->SetVariables(u_serial);

  nlp_.AddVariableSet(variables);

  // find solution
  ipopt_.Solve(nlp_);

  // store rresults
  Eigen::VectorXd vals = nlp_.GetOptVariables()->GetValues();
  u_.row(0).head(n) = vals.head(n);
  u_.row(1).head(n) = vals.tail(n);
};

void RecidingHorizonController::generatePath() {
  x_ = dynamics_->rk4(u_);
}

nav_msgs::msg::Path RecidingHorizonController::getPath(
    [[maybe_unused]] const std::size_t granulaty) {
  nav_msgs::msg::Path path;

  auto populatePathMsg = [&](const auto& x) {
    geometry_msgs::msg::PoseStamped stamped_pose;
    stamped_pose.header.frame_id = "base_link";
    stamped_pose.pose.position.x = x[1];
    stamped_pose.pose.position.y = x[2];
    const auto q = tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), x[3]);
    stamped_pose.pose.orientation = tf2::toMsg(q);
    return stamped_pose;
  };

  std::transform(x_.colwise().begin(), x_.colwise().end(), std::back_inserter(path.poses),
                 populatePathMsg);
  path.header.frame_id = "base_link";
  // path.header.stamp = pose.header.stamp;

  return path;
}

geometry_msgs::msg::Twist RecidingHorizonController::getVelocityCommand() {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = x_.row(0)[2];
  twist.angular.z = x_.row(3)[2];
  return twist;
}

void RecidingHorizonController::setChaseWeights() {
  Dynamics::MatrixWeights W;
  W.diagonal()[0] = 10.0;
  W.diagonal()[1] = 100000.0;
  W.diagonal()[2] = 100000.0;
  W.diagonal()[3] = 0.0;
  W.diagonal()[4] = 10000000.0;
  dynamics_->setWeightMatrix(W);
}

void RecidingHorizonController::setPositioningWeights() {
  Dynamics::MatrixWeights W;
  W.diagonal()[0] = 10.0;
  W.diagonal()[1] = 100000.0;
  W.diagonal()[2] = 100000.0;
  W.diagonal()[3] = 10.0;
  W.diagonal()[4] = 100000.0;
  dynamics_->setWeightMatrix(W);
}

void RecidingHorizonController::setState(const geometry_msgs::msg::Twist& twist_start,
                                         const geometry_msgs::msg::PoseStamped& pose_end,
                                         const geometry_msgs::msg::Twist& twist_end) {
  Dynamics::VectorState x0;
  x0[0] = twist_start.linear.x;
  x0[1] = 0.0;
  x0[2] = 0.0;
  x0[3] = twist_start.angular.z;
  x0[4] = 0.0;

  Dynamics::VectorState xf;
  xf[0] = twist_end.linear.x;
  xf[1] = pose_end.pose.position.x;
  xf[2] = pose_end.pose.position.y;
  xf[3] = 0.0;
  xf[4] = tf2::getYaw(pose_end.pose.orientation);

  dynamics_->setupState(x0, xf);
}

void RecidingHorizonController::setNIter(const std::size_t max_iter) {
  max_iter_ = max_iter;
}

};  // namespace receding_horizon_controller
