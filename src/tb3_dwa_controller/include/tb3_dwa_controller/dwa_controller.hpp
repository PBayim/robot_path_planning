#ifndef TB3_DWA_CONTROLLER__DWA_CONTROLLER_HPP_
#define TB3_DWA_CONTROLLER__DWA_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace tb3_dwa_controller
{

class DWAController : public nav2_core::Controller
{
public:
  DWAController() = default;
  ~DWAController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  struct Trajectory
  {
    double v{0.0};
    double w{0.0};
    double score{-std::numeric_limits<double>::infinity()};
    bool valid{false};
    std::vector<geometry_msgs::msg::Pose2D> poses;
  };

  void computeDynamicWindow(
    const geometry_msgs::msg::Twist & velocity,
    double & v_min, double & v_max,
    double & w_min, double & w_max) const;

  std::vector<geometry_msgs::msg::Pose2D> simulateTrajectory(double v, double w) const;

  void scoreTrajectory(
    Trajectory & trajectory,
    const geometry_msgs::msg::Pose2D & local_goal_robot_frame,
    const geometry_msgs::msg::Pose2D & robot_pose) const;

  geometry_msgs::msg::Pose2D computeLocalGoal(
    const geometry_msgs::msg::PoseStamped & robot_pose) const;

  double trajectoryClearanceCost(
    const std::vector<geometry_msgs::msg::Pose2D> & poses,
    const geometry_msgs::msg::Pose2D & robot_pose) const;

  void updateOscillationLock(const geometry_msgs::msg::Pose2D & robot_pose, double v, double w);

  bool violatesOscillationLock(double v, double w) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  mutable nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
  collision_checker_;
  rclcpp::Logger logger_{rclcpp::get_logger("DWAController")};
  rclcpp::Clock::SharedPtr clock_;

  nav_msgs::msg::Path global_plan_;

  // --- Velocity / acceleration limits ---
  double max_vel_x_{0.26};
  double min_vel_x_{0.0};
  double max_vel_theta_{1.0};
  double min_vel_theta_{-1.0};
  double acc_lim_x_{2.5};
  double decel_lim_x_{2.5};
  double acc_lim_theta_{3.2};

  // --- Dynamic window / sampling ---
  double dynamic_window_dt_{0.2};
  int vx_samples_{10};
  int vtheta_samples_{20};

  // --- Trajectory rollout ---
  double sim_time_{1.5};
  double sim_granularity_{0.1};

  // --- Scoring weights ---
  double heading_weight_{0.8};
  double clearance_weight_{0.3};
  double velocity_weight_{0.2};

  // --- Local sub-goal selection ---
  double lookahead_distance_{1.0};
  // Slides the local goal past blocked plan segments so the heading critic
  // aims around a moving obstacle rather than into it.
  double obstacle_lookahead_extend_dist_{2.0};
  // Ramps desired speed to zero within this distance of the final goal.
  double goal_slowdown_distance_{0.5};

  // --- Oscillation guard ---
  double oscillation_reset_dist_{0.05};
  double oscillation_reset_angle_{0.2};
  bool x_lock_active_{false};
  int x_lock_sign_{0};
  geometry_msgs::msg::Pose2D x_lock_pose_;
  bool theta_lock_active_{false};
  int theta_lock_sign_{0};
  geometry_msgs::msg::Pose2D theta_lock_pose_;

  double speed_limit_{-1.0};
  bool speed_limit_is_percentage_{false};

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_plan_pub_;
};

}  // namespace tb3_dwa_controller

#endif  // TB3_DWA_CONTROLLER__DWA_CONTROLLER_HPP_
