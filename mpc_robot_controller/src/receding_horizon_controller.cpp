#include <receding_horizon_controller.hpp>

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

RecidingHorizonController::RecidingHorizonController(const std::shared_ptr<Dynamics>& dynamics)
    : dynamics_(dynamics) {

  constraint_ = std::make_shared<controller_nlp::ControllerConstraint>(dynamics_);
  cost_ = std::make_shared<controller_nlp::ControllerCost>(dynamics_);
  nlp_.AddConstraintSet(constraint_);
  nlp_.AddCostSet(cost_);
};

void RecidingHorizonController::roll(const std::size_t n) {
  u_.leftCols(n) = u_.middleCols(1, n);
  if (n < u_.cols()) {
    u_.rightCols(u_.cols() - n) = 0.0;
  }
};

void RecidingHorizonController::predict(const std::size_t n) {
  auto variables = std::make_shared<controller_nlp::ControllerVariables>(dynamics_, n);

  // warm start
  const auto u_serial = ifopt::Component::VectorXd(u_.cols() * n);
  u_serial.head(n / u_.cols()) = u_.row(0);
  u_serial.tail(n / u_.cols()) = u_.row(1);
  variables.SetVariables(u_serial);

  // find solution
  nlp_.AddVariableSet(variables);
  ipopt_.SetOption("max_iter", max_iter_);
  ipopt_.Solve(nlp_);

  // store rresults
  Eigen::VectorXd vals = nlp.GetOptVariables()->GetValues();
  u_.row(0) = vals.head(n / u_.cols());
  u_.row(1) = vals.tail(n / u_.cols());
};

void RecidingHorizonController::generatePath() {
  x_ = dynamics_->rk4(u_);
}

nav_msgs::msg::Path RecidingHorizonController::getPath(const std::size_t granulaty) {
  nav_msgs::msg::Path path;

  auto populatePathMsg = [&](const auto& state) {
    geometry_msgs::msg::PoseStamped stamped_pose;
    stamped_pose.header.frame_id = "base_link";
    stamped_pose.pose.position.x = x[1];
    stamped_pose.pose.position.y = x[2];
    stamped_pose.pose.orientation = tf2::Quaternion(tf2::Vector3(0, 0, 1), x[3]);
    return stamped_pose;
  };

  std::transform(x_.rowwise().begin(), x_.rowwise().end(), std::back_inserter(path.poses),
                 populatePathMsg);
  path.header.frame_id = "base_link";
  path.header.stamp = pose.header.stamp;

  return path;
}

geometry_msgs::msg::Twist RecidingHorizonController::getVelocityCommand() {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = u_.row(0)[0];
  twist.angular.z = u_.row(0)[1];
  return twist;
}

void RecidingHorizonController::setChaseWeights() {
  Dynamics::MatrixWeights W;
  W.diagonal()[0] = 0.0;
  W.diagonal()[1] = 100.0;
  W.diagonal()[2] = 100.0;
  W.diagonal()[3] = 0.0;
  W.diagonal()[4] = 0.0;
  dynamics_->setWeightMatrix(W);
}

void RecidingHorizonController::setPositioningWeights() {
  Dynamics::MatrixWeights W;
  W.diagonal()[0] = 1.0;
  W.diagonal()[1] = 100.0;
  W.diagonal()[2] = 100.0;
  W.diagonal()[3] = 1.0;
  W.diagonal()[4] = 10.0;
  dynamics_->setWeightMatrix(W);
}

void RecidingHorizonController::setMap(const nav2_costmap_2d::Costmap2D* map) {}

void RecidingHorizonController::setFinalState(const geometry_msgs::msg::PoseStamped& pose) {
  Dynamics::VectorState x0;
  x0 = Dynamics::VectorState::Zero();

  Dynamics::VectorState xf;
  xf[0] = 0.0;
  xf[1] = pose.pose.position.x;
  xf[2] = pose.pose.position.y;
  xf[3] = 0.0;
  xf[4] = tf2::getYaw(pose.orientation);
  dynamics_->setupState(x0, xf);
}

void RecidingHorizonController::setNIter(const std::size_t max_iter) {
  max_iter_ = max_iter
}

};  // namespace receding_horizon_controller
