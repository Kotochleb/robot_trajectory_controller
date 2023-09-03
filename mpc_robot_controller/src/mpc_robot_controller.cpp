#include <mpc_robot_controller/mpc_robot_controller.hpp>

#include <algorithm>
#include <memory>
#include <string>

#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <nav2_core/exceptions.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <nav2_util/node_utils.hpp>

#include <mpc_robot_controller/diff_drive_dynamics.hpp>
#include <mpc_robot_controller/receding_horizon_controller.hpp>
#include <mpc_robot_controller/robot_dynamics.hpp>

namespace mpc_robot_controller {

using rcl_interfaces::msg::ParameterType;

void MPCRobotController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                                   std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                                   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {

  node_ = parent;
  auto node = node_.lock();

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  diff_drive_dynamics::DiffDriveParams params;

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".time_steps",
                                               rclcpp::ParameterValue(50));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_iter",
                                               rclcpp::ParameterValue(10000));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".model_dt",
                                               rclcpp::ParameterValue(0.1));

  const auto lim_vel_pref = plugin_name_ + ".limits.velocity";
  nav2_util::declare_parameter_if_not_declared(node, lim_vel_pref + ".forward",
                                               rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, lim_vel_pref + ".backward",
                                               rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, lim_vel_pref + ".angular",
                                               rclcpp::ParameterValue(3.14));

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".limits.acceleration",
                                               rclcpp::ParameterValue(1.0));

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance",
                                               rclcpp::ParameterValue(0.01));

  int ts, mi;
  node->get_parameter(plugin_name_ + ".time_steps", ts);
  node->get_parameter(plugin_name_ + ".max_iter", mi);
  time_steps_ = static_cast<std::size_t>(ts);
  max_iter_ = static_cast<std::size_t>(mi);

  node->get_parameter(plugin_name_ + ".model_dt", model_dt_);
  node->get_parameter(lim_vel_pref + ".forward", params.velocity.forward);
  node->get_parameter(lim_vel_pref + ".backward", params.velocity.backward);
  node->get_parameter(lim_vel_pref + ".angular", params.velocity.angular);
  node->get_parameter(plugin_name_ + ".limits.acceleration", params.acceleration);

  double transform_tolerance;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);

  double controller_frequency;
  node->get_parameter("controller_frequency", controller_frequency);
  // cut off the computation after 80% of the loop time
  const double max_cpu_time = 1.0 / controller_frequency * 0.8;

  params.dt = model_dt_;
  rd_ = std::make_shared<diff_drive_dynamics::DiffDriveDynamics>(params);
  rhc_ = std::make_unique<receding_horizon_controller::RecidingHorizonController>(
      rd_, time_steps_, max_iter_, max_cpu_time);

  targer_pose_pub_ =
      node->create_publisher<geometry_msgs::msg::PoseStamped>("local_plan_goal_pose", 1);
  local_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
}

void MPCRobotController::cleanup() {
  RCLCPP_INFO(logger_,
              "Cleaning up controller: %s of type mpc_robot_controller::MPCRobotController",
              plugin_name_.c_str());
  local_pub_.reset();
  targer_pose_pub_.reset();
}

void MPCRobotController::activate() {
  RCLCPP_INFO(logger_, "Activating controller: %s of type mpc_robot_controller::MPCRobotController",
              plugin_name_.c_str());
  local_pub_->on_activate();
  targer_pose_pub_->on_activate();
}

void MPCRobotController::deactivate() {
  RCLCPP_INFO(logger_,
              "Dectivating controller: %s of type mpc_robot_controller::MPCRobotController",
              plugin_name_.c_str());
  local_pub_->on_deactivate();
  targer_pose_pub_->on_deactivate();
}

geometry_msgs::msg::TwistStamped MPCRobotController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& twist,
    [[maybe_unused]] nav2_core::GoalChecker* goal_checker) {

  geometry_msgs::msg::PoseStamped goal;
  tf_->transform(getGoal(pose), goal, "base_link", transform_tolerance_);

  const auto costmap = costmap_ros_->getCostmap();

  diff_drive_dynamics::CostMap robot_map;
  robot_map.resolution = costmap->getResolution();
  robot_map.cells_x = costmap->getSizeInCellsX();
  robot_map.cells_y = costmap->getSizeInCellsY();
  robot_map.map = costmap->getCharMap();
  rd_->setMap(robot_map);

  geometry_msgs::msg::Twist goal_twist;

  const auto thresh = (costmap->getSizeInMetersX() + costmap->getSizeInMetersY()) / 2.0 * 0.8;
  if (std::hypot(goal.pose.position.x, goal.pose.position.y) > thresh * 0.6) {
    rhc_->setChaseWeights();
    goal_twist.linear.x = rd_->getVelocityForwardLimit();
  } else {
    rhc_->setPositioningWeights();
  }

  rhc_->setState(twist, goal, goal_twist);
  const auto n = time_steps_;
  rhc_->roll(n);
  rhc_->predict(n);
  rhc_->generatePath();

  const auto path = rhc_->getPath(n);

  local_pub_->publish(path);
  targer_pose_pub_->publish(goal);

  // Create and publish a TwistStamped message with the desired velocity
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist = rhc_->getVelocityCommand();

  return cmd_vel;
}

void MPCRobotController::setPlan(const nav_msgs::msg::Path& path) {
  global_plan_ = path;
}

void MPCRobotController::setSpeedLimit([[maybe_unused]] const double& speed_limit,
                                       [[maybe_unused]] const bool& percentage){};

geometry_msgs::msg::PoseStamped MPCRobotController::getGoal(
    const geometry_msgs::msg::PoseStamped& robot_pose) {
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length!");
  }

  geometry_msgs::msg::PoseStamped global_robot_pose;

  tf_->transform(robot_pose, global_robot_pose, global_plan_.header.frame_id, transform_tolerance_);

  const auto costmap = costmap_ros_->getCostmap();
  const auto thresh = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
                      costmap->getResolution() / 2.0;

  geometry_msgs::msg::PoseStamped end_pose;
  end_pose.header.stamp = robot_pose.header.stamp;
  end_pose.header.frame_id = global_plan_.header.frame_id;

  if (global_plan_.poses.size() > 1) {
    for (std::size_t i = 0; i < global_plan_.poses.size() - 1; i++) {
      const auto& pose = global_plan_.poses[i];
      if (nav2_util::geometry_utils::euclidean_distance(pose, global_robot_pose) > thresh) {
        const auto& next_pose = global_plan_.poses[i + 1];
        const double x = next_pose.pose.position.x - pose.pose.position.x;
        const double y = next_pose.pose.position.y - pose.pose.position.y;
        const double yaw = std::atan2(y, x);
        const auto q = tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw);

        end_pose.pose.position = pose.pose.position;
        end_pose.pose.orientation = tf2::toMsg(q);
        return end_pose;
      }
    }
  }

  end_pose.pose = global_plan_.poses.back().pose;
  return end_pose;
}

}  // namespace mpc_robot_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(mpc_robot_controller::MPCRobotController, nav2_core::Controller)
