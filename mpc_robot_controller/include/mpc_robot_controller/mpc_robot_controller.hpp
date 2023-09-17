#ifndef __MPC_ROBOT_CONTROLLER_HPP__
#define __MPC_ROBOT_CONTROLLER_HPP__

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <pluginlib/class_loader.hpp>

#include <tf2/time.h>

#include <nav2_core/controller.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <mpc_robot_controller/diff_drive_dynamics.hpp>
#include <mpc_robot_controller/receding_horizon_controller.hpp>
#include <mpc_robot_controller/robot_dynamics.hpp>

#include <mpc_robot_controller/robot_dynamics.hpp>

namespace mpc_robot_controller {

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

 protected:
  std::size_t time_steps_;
  std::size_t max_iter_;
  double model_dt_;
  double goal_dist_thresh_;
  bool debug_;
  unsigned path_depth_;
  unsigned path_depth_idx_;
  unsigned recovery_cnt_{0};
  std::vector<geometry_msgs::msg::Twist> twist_mem_;
  nav_msgs::msg::Path last_path_;
  geometry_msgs::msg::PoseStamped last_goal_;

  robot_dynamics::ReduceMap generateReducedCostmap();

  std::shared_ptr<diff_drive_dynamics::DiffDriveDynamics> rd_;
  std::shared_ptr<receding_horizon_controller::RecidingHorizonController> rhc_;

  std::string plugin_name_;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPCRobotController")};
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  tf2::Duration transform_tolerance_;

  nav_msgs::msg::Path global_plan_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>>
      targer_pose_pub_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>>
      map_gradient_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>>
      continous_costmap_pub_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
      reduced_costmap_pub_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  geometry_msgs::msg::PoseStamped getGoal(const geometry_msgs::msg::PoseStamped& robot_pose);
  geometry_msgs::msg::PoseArray getMapGradient(const geometry_msgs::msg::PoseStamped& robot_pose);
  //   nav_msgs::msg::OccupancyGrid getContinousMap(const geometry_msgs::msg::PoseStamped& robot_pose);
  visualization_msgs::msg::MarkerArray getCostmapMarkerArray(const robot_dynamics::ReduceMap& rm,
                                                             const std_msgs::msg::Header& header);
  bool isColliding(const nav_msgs::msg::Path& path);
  geometry_msgs::msg::Twist getRecoveryDirection(const geometry_msgs::msg::PoseStamped& robot_pose);
};

}  // namespace mpc_robot_controller

#endif  // __MPC_ROBOT_CONTROLLER_HPP__
