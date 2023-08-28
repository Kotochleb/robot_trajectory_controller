#ifndef __MPC_ROBOT_CONTROLLER_HPP__
#define __MPC_ROBOT_CONTROLLER_HPP__

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

#include <mpc_robot_controller/diff_drive_dynamics.hpp>
#include <mpc_robot_controller/receding_horizon_controller.hpp>
#include <mpc_robot_controller/robot_dynamics.hpp>

namespace mpc_robot_controller {

struct motion_limit_t {
  double forward;
  double backward;
  double angular;
};

class MPCRobotController : public nav2_core::Controller {
 public:
  MPCRobotController() = default;
  ~MPCRobotController() override = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity,
      nav2_core::GoalChecker* goal_checker) override;

  void setPlan(const nav_msgs::msg::Path& path) override;

  void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

  double getTimeToEscapeMap(const double velocity) {
    const nav2_costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    const double circ_r = std::max(costmap->getSizeInMetersX, costmap->getSizeInMetersY);

    if (std::fabs(velocity - velocity_lim_.forward) <= std::numeric_limits<double>::epsilon()) {
      return circ_r / velocity;
    }
    const double acc_dist =
        std::pow(velocity_lim_.forward - velocity, 2) / (2.0 * acceleration_lim_.forward);
    if (acc_dist > circ_r) {
      const double acc_time = (velocity_lim_.forward - velocity) / acceleration_lim_.forward;
      const double max_speed_time = (circ_r - acc_dist) / velocity_lim_.forward;
      return acc_time + max_speed_time;
    } else {
      const auto delta =
          std::sqrt(std::pow(velocity, 2) + 2.0 * acceleration_lim_.forward * circ_r);
      return (-velocity + delta) / acceleration_lim_.forward;
    }
  }

 protected:
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped& pose);
  double getTimeContraing(const double velocity);

  bool transformPose(const std::shared_ptr<tf2_ros::Buffer> tf, const std::string frame,
                     const geometry_msgs::msg::PoseStamped& in_pose,
                     geometry_msgs::msg::PoseStamped& out_pose,
                     const rclcpp::Duration& transform_tolerance) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPCRobotController")};
  rclcpp::Clock::SharedPtr clock_;

  std::size_t time_steps_;
  std::size_t max_iter_;
  double mode_dt_;
  motion_limit_t velocity_lim_;
  motion_limit_t acceleration_lim_;
  motion_limit_t deceleration_lim_;
  rclcpp::Duration transform_tolerance_{0, 0};

  std::shared_ptr<diff_drive_dynamics::DiffDriveDynamics> rd_;
  std::shared_ptr<receding_horizon_controller::RecidingHorizonController> rhc_;

  nav_msgs::msg::Path global_plan_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_pub_;
};

}  // namespace mpc_robot_controller

#endif  // __MPC_ROBOT_CONTROLLER_HPP__
