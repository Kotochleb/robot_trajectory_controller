#include <mpc_robot_controller/receding_horizon_controller.hpp>

#include <memory>

#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
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
      max_iter_(static_cast<int>(max_iter)),
      max_cpu_time_(max_cpu_time),
      u_(Dynamics::MatrixControl(Dynamics::MatrixControl::RowsAtCompileTime, max_horizon)) {

  constraint_ = std::make_shared<controller_nlp::ControllerConstraint>(dynamics_);
  cost_ = std::make_shared<controller_nlp::ControllerCost>(dynamics_);

  resetOptimuizer();
};

void RecidingHorizonController::setMap(const robot_dynamics::ReduceMap& rm) {
  dynamics_->setSigmaMap(rm);
}

void RecidingHorizonController::roll(const std::size_t /*n*/) {
  const Eigen::Index c = u_.cols();
  if (c <= 0) {
    throw std::runtime_error("Can not roll empty control input!");
  }

  u_.leftCols(c - 1) = u_.rightCols(c - 1);
  u_.rightCols(0) = u_.rightCols(1);

  // if (n >= std::size_t(c)) {
  //   u_.leftCols(c - 1) = u_.middleCols(1, c - 2);
  //   u_.col(c - 1) = u_.col(c - 2);
  // } else {
  //   u_.leftCols(n) = u_.middleCols(1, n);
  //   u_.rightCols(c - n).setConstant(0.0);
  // }
};

void RecidingHorizonController::clearControlMemory() {
  u_.setZero();
}

void RecidingHorizonController::randomizeControlMemory() {
  u_.setRandom();
  u_.row(0) *= dynamics_->getLinearAccelerationLimit() / 2.0;
  u_.row(1) *= dynamics_->getAngularAccelerationLimit() / 2.0;
}

void RecidingHorizonController::resetOptimuizer() {
  if (!ipopt_) {
    ipopt_.reset();
  }
  ipopt_ = std::make_unique<ifopt::IpoptSolver>();

  ipopt_->SetOption("max_iter", max_iter_);
  ipopt_->SetOption("print_level", 0);
  ipopt_->SetOption("max_cpu_time", max_cpu_time_);
  // ipopt_->SetOption("acceptable_tol", 1e-4);


  // ipopt_->SetOption("derivative_test", "first-order");
  // ipopt_->SetOption("derivative_test_tol", 1e-4);
}

void RecidingHorizonController::setWarmStart() {
  ipopt_->SetOption("warm_start_init_point", "yes");
  ipopt_->SetOption("warm_start_bound_push", 1e-12);
  ipopt_->SetOption("warm_start_mult_bound_push", 1e-12);
  ipopt_->SetOption("warm_start_bound_frac", 1e-12);
  ipopt_->SetOption("warm_start_slack_bound_frac", 1e-12);
  ipopt_->SetOption("warm_start_slack_bound_push", 1e-12);
  ipopt_->SetOption("mu_init", 1e-6);
}

void RecidingHorizonController::setColdStart() {
  ipopt_->SetOption("warm_start_init_point", "no");
  ipopt_->SetOption("warm_start_bound_push", 1e-6);
  ipopt_->SetOption("warm_start_mult_bound_push", 1e-6);
  ipopt_->SetOption("warm_start_bound_frac", 1e-6);
  ipopt_->SetOption("warm_start_slack_bound_frac", 1e-6);
  ipopt_->SetOption("warm_start_slack_bound_push", 1e-6);
  ipopt_->SetOption("mu_init", 0.1);
}

bool RecidingHorizonController::predict(const std::size_t n) {
  ifopt::Problem nlp;
  nlp.AddConstraintSet(constraint_);
  nlp.AddCostSet(cost_);
  auto variables = std::make_shared<controller_nlp::ControllerVariables>(dynamics_, n);

  // warm start
  auto u_serial = ifopt::Component::VectorXd(u_.rows() * n);
  u_serial.head(n / u_.rows()) = u_.row(0);
  u_serial.tail(n / u_.rows()) = u_.row(1);
  variables->SetVariables(u_serial);

  nlp.AddVariableSet(variables);

  // find solution
  ipopt_->Solve(nlp);

  // store rresults
  if (ok_responses_.find(ipopt_->GetReturnStatus()) != ok_responses_.end()) {
    Eigen::VectorXd vals = nlp.GetOptVariables()->GetValues();
    u_.row(0).head(n) = vals.head(n);
    u_.row(1).head(n) = vals.tail(n);
    return true;
  }
  return false;
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
  return path;
}

std::vector<geometry_msgs::msg::Twist> RecidingHorizonController::getVelocityCommands(
    const unsigned n) {
  std::vector<geometry_msgs::msg::Twist> twist_mem;
  twist_mem.reserve(n);
  for (unsigned i = 0; i < n; i++) {
    const auto& x = x_.col(i + 1);
    twist_mem[i].linear.x = x[0];
    twist_mem[i].angular.z = x[3];
  }
  return twist_mem;
}

void RecidingHorizonController::setControlWeights(const std::vector<double> r_vect) {
  Dynamics::MatrixControlWeights R;
  assert(R.diagonal().size() == r_vect.size());
  for (unsigned i = 0; i < r_vect.size(); i++) {
    R.diagonal()[i] = r_vect[i];
  }
  dynamics_->setControlWeightMatrix(R);
}

void RecidingHorizonController::setStateWeights(const std::vector<double> w_vect) {
  Dynamics::MatrixStateWeights W;
  assert(W.diagonal().size() == w_vect.size());
  for (unsigned i = 0; i < w_vect.size(); i++) {
    W.diagonal()[i] = w_vect[i];
  }
  dynamics_->setStateWeightMatrix(W);
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
  max_iter_ = static_cast<int>(max_iter);
}

};  // namespace receding_horizon_controller
