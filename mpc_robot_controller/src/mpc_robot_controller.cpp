#include <algorithm>
#include <memory>
#include <string>

#include "mpc_robot_controller/mpc_robot_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

#include <diff_drive_dynamics.hpp>
#include <receding_horizon_controller.hpp>
#include <robot_dynamics.hpp>
#include <utils.hpp>

using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;

namespace mpc_robot_controller {

void MPCRobotController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
    const std::shared_ptr<tf2_ros::Buffer>& tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>& costmap_ros) {

  node_ = parent;
  auto node = node_.lock();

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  const auto lim_vel_pref = plugin_name_ + ".limits.velocity";
  const auto lim_acc_pref = plugin_name_ + ".limits.acceleration";
  const auto lim_dacc_pref = plugin_name_ + ".limits.deceleration";

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".time_steps",
                                               rclcpp::ParameterValue(50));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".model_dt",
                                               rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_iter",
                                               rclcpp::ParameterValue(20));

  nav2_util::declare_parameter_if_not_declared(node, lim_vel_pref + ".forward",
                                               rclcpp::ParameterValue(20));
  nav2_util::declare_parameter_if_not_declared(node, lim_vel_pref + ".backward",
                                               rclcpp::ParameterValue(20));
  nav2_util::declare_parameter_if_not_declared(node, lim_vel_pref + ".angular",
                                               rclcpp::ParameterValue(20));

  nav2_util::declare_parameter_if_not_declared(node, lim_acc_pref + ".forward",
                                               rclcpp::ParameterValue(20));
  nav2_util::declare_parameter_if_not_declared(node, lim_acc_pref + ".backward",
                                               rclcpp::ParameterValue(20));
  nav2_util::declare_parameter_if_not_declared(node, lim_acc_pref + ".angnular",
                                               rclcpp::ParameterValue(20));

  nav2_util::declare_parameter_if_not_declared(node, lim_dacc_pref + ".forward",
                                               rclcpp::ParameterValue(20));
  nav2_util::declare_parameter_if_not_declared(node, lim_dacc_pref + ".backward",
                                               rclcpp::ParameterValue(20));
  nav2_util::declare_parameter_if_not_declared(node, lim_dacc_pref + ".angnular",
                                               rclcpp::ParameterValue(20));

  node->get_parameter(plugin_name_ + ".time_steps", time_steps_);
  node->get_parameter(plugin_name_ + ".model_dt", mode_dt_);
  node->get_parameter(plugin_name_ + ".max_iter", max_iter_);

  node->get_parameter(lim_vel_pref + ".forward", velocity_lim_.forward);
  node->get_parameter(lim_vel_pref + ".backward", velocity_lim_.backward);
  node->get_parameter(lim_vel_pref + ".angular", velocity_lim_.angular);

  node->get_parameter(lim_vel_pref + ".angular", acceleration_lim_.forward);
  node->get_parameter(lim_acc_pref + ".forward", acceleration_lim_.backward);
  node->get_parameter(lim_acc_pref + ".backward", acceleration_lim_.angular);

  node->get_parameter(lim_acc_pref + ".angnular", deceleration_lim_.forward);
  node->get_parameter(lim_dacc_pref + ".forward", deceleration_lim_.backward);
  node->get_parameter(lim_dacc_pref + ".backward", deceleration_lim_.angular);

  double transform_tolerance;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

  rd_ = std::make_shared<diff_drive_dynamics::DiffDriveDynamics>(0.1, 0.1, 0.1);
  rhc_ = std::make_unique<receding_horizon_controller::RecidingHorizonController>(rd_);

  local_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
}

void MPCRobotController::cleanup() {
  RCLCPP_INFO(logger_,
              "Cleaning up controller: %s of type mpc_robot_controller::MPCRobotController",
              plugin_name_.c_str());
  local_pub_.reset();
}

void MPCRobotController::activate() {
  RCLCPP_INFO(logger_, "Activating controller: %s of type mpc_robot_controller::MPCRobotController",
              plugin_name_.c_str());
  local_pub_->on_activate();
}

void MPCRobotController::deactivate() {
  RCLCPP_INFO(logger_,
              "Dectivating controller: %s of type mpc_robot_controller::MPCRobotController",
              plugin_name_.c_str());
  local_pub_->on_deactivate();
}

geometry_msgs::msg::TwistStamped MPCRobotController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& twist) {

  auto transformed_plan = transformGlobalPlan(pose);
  const auto map = std::make_unique<nav2_costmap_2d::Costmap2D>(costmap_ros_->getCostmap());

  const std::size_t n = std::size_t(getTimeToEscapeMap(std::abs(twist.linear.x)) / mode_dt_) + 1;

  if (isInsideMap(transformed_plan.poses.back().pose.position)) {
    rhc_->setPositioningWeights();
  } else {
    rhc_->setChaseWeights();
  }

  rhc_->roll(n);
  rhc_->predict(n);
  rhc_->generatePath();

  const auto path = rhc->getPath(n);
  local_pub_->publish(path);

  // Create and publish a TwistStamped message with the desired velocity
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist = rhc->getVelocityCommand();

  return cmd_vel;
}

void MPCRobotController::setPlan(const nav_msgs::msg::Path& path) {
  global_plan_ = path;
}

void setSpeedLimit(const double& speed_limit, const bool& percentage){};

}  // namespace mpc_robot_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(mpc_robot_controller::MPCRobotController, nav2_core::Controller)
