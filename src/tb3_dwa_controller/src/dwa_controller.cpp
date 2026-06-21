#include "tb3_dwa_controller/dwa_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "angles/angles.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace tb3_dwa_controller
{

namespace
{
double angleDifference(double a, double b)
{
  double diff = std::fabs(angles::shortest_angular_distance(a, b));
  return diff;
}

// Treats |x| < eps as zero to prevent the oscillation lock from arming on numerical noise.
int signOf(double x, double eps)
{
  if (x > eps) {return 1;}
  if (x < -eps) {return -1;}
  return 0;
}

constexpr double kSignEps = 1e-3;
}  // namespace

void DWAController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  auto declare = [&](const std::string & param, auto default_value) {
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + "." + param, rclcpp::ParameterValue(default_value));
    };

  declare("max_vel_x", max_vel_x_);
  declare("min_vel_x", min_vel_x_);
  declare("max_vel_theta", max_vel_theta_);
  declare("min_vel_theta", min_vel_theta_);
  declare("acc_lim_x", acc_lim_x_);
  declare("decel_lim_x", decel_lim_x_);
  declare("acc_lim_theta", acc_lim_theta_);
  declare("dynamic_window_dt", dynamic_window_dt_);
  declare("vx_samples", vx_samples_);
  declare("vtheta_samples", vtheta_samples_);
  declare("sim_time", sim_time_);
  declare("sim_granularity", sim_granularity_);
  declare("heading_weight", heading_weight_);
  declare("clearance_weight", clearance_weight_);
  declare("velocity_weight", velocity_weight_);
  declare("lookahead_distance", lookahead_distance_);
  declare("obstacle_lookahead_extend_dist", obstacle_lookahead_extend_dist_);
  declare("goal_slowdown_distance", goal_slowdown_distance_);
  declare("oscillation_reset_dist", oscillation_reset_dist_);
  declare("oscillation_reset_angle", oscillation_reset_angle_);

  node->get_parameter(plugin_name_ + ".max_vel_x", max_vel_x_);
  node->get_parameter(plugin_name_ + ".min_vel_x", min_vel_x_);
  node->get_parameter(plugin_name_ + ".max_vel_theta", max_vel_theta_);
  node->get_parameter(plugin_name_ + ".min_vel_theta", min_vel_theta_);
  node->get_parameter(plugin_name_ + ".acc_lim_x", acc_lim_x_);
  node->get_parameter(plugin_name_ + ".decel_lim_x", decel_lim_x_);
  node->get_parameter(plugin_name_ + ".acc_lim_theta", acc_lim_theta_);
  node->get_parameter(plugin_name_ + ".dynamic_window_dt", dynamic_window_dt_);
  node->get_parameter(plugin_name_ + ".vx_samples", vx_samples_);
  node->get_parameter(plugin_name_ + ".vtheta_samples", vtheta_samples_);
  node->get_parameter(plugin_name_ + ".sim_time", sim_time_);
  node->get_parameter(plugin_name_ + ".sim_granularity", sim_granularity_);
  node->get_parameter(plugin_name_ + ".heading_weight", heading_weight_);
  node->get_parameter(plugin_name_ + ".clearance_weight", clearance_weight_);
  node->get_parameter(plugin_name_ + ".velocity_weight", velocity_weight_);
  node->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
  node->get_parameter(
    plugin_name_ + ".obstacle_lookahead_extend_dist",
    obstacle_lookahead_extend_dist_);
  node->get_parameter(plugin_name_ + ".goal_slowdown_distance", goal_slowdown_distance_);
  node->get_parameter(plugin_name_ + ".oscillation_reset_dist", oscillation_reset_dist_);
  node->get_parameter(plugin_name_ + ".oscillation_reset_angle", oscillation_reset_angle_);

  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);

  RCLCPP_INFO(
    logger_,
    "DWAController '%s' configured: max_vel_x=%.2f max_vel_theta=%.2f "
    "sim_time=%.2f samples=%dx%d weights(head/clear/vel)=%.2f/%.2f/%.2f "
    "goal_slowdown_distance=%.2f",
    plugin_name_.c_str(), max_vel_x_, max_vel_theta_, sim_time_,
    vx_samples_, vtheta_samples_, heading_weight_, clearance_weight_, velocity_weight_,
    goal_slowdown_distance_);
}

void DWAController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up DWAController '%s'", plugin_name_.c_str());
  local_plan_pub_.reset();
}

void DWAController::activate()
{
  RCLCPP_INFO(logger_, "Activating DWAController '%s'", plugin_name_.c_str());
  local_plan_pub_->on_activate();
  x_lock_active_ = false;
  theta_lock_active_ = false;
}

void DWAController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating DWAController '%s'", plugin_name_.c_str());
  local_plan_pub_->on_deactivate();
}

void DWAController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void DWAController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

void DWAController::computeDynamicWindow(
  const geometry_msgs::msg::Twist & velocity,
  double & v_min, double & v_max,
  double & w_min, double & w_max) const
{
  double abs_max_vel_x = max_vel_x_;
  if (speed_limit_ > 0.0) {
    abs_max_vel_x = speed_limit_is_percentage_ ?
      max_vel_x_ * speed_limit_ / 100.0 :
      std::min(max_vel_x_, speed_limit_);
  }

  v_min = std::max(min_vel_x_, velocity.linear.x - decel_lim_x_ * dynamic_window_dt_);
  v_max = std::min(abs_max_vel_x, velocity.linear.x + acc_lim_x_ * dynamic_window_dt_);

  w_min = std::max(min_vel_theta_, velocity.angular.z - acc_lim_theta_ * dynamic_window_dt_);
  w_max = std::min(max_vel_theta_, velocity.angular.z + acc_lim_theta_ * dynamic_window_dt_);
}

std::vector<geometry_msgs::msg::Pose2D> DWAController::simulateTrajectory(double v, double w) const
{
  std::vector<geometry_msgs::msg::Pose2D> poses;
  const int steps = std::max(1, static_cast<int>(std::round(sim_time_ / sim_granularity_)));
  poses.reserve(steps);

  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;

  for (int i = 1; i <= steps; ++i) {
    x += v * std::cos(theta) * sim_granularity_;
    y += v * std::sin(theta) * sim_granularity_;
    theta += w * sim_granularity_;

    geometry_msgs::msg::Pose2D p;
    p.x = x;
    p.y = y;
    p.theta = theta;
    poses.push_back(p);
  }

  return poses;
}

double DWAController::trajectoryClearanceCost(
  const std::vector<geometry_msgs::msg::Pose2D> & poses_robot_frame,
  const geometry_msgs::msg::Pose2D & robot_pose) const
{
  const double rx = robot_pose.x;
  const double ry = robot_pose.y;
  const double ryaw = robot_pose.theta;
  const double cos_r = std::cos(ryaw);
  const double sin_r = std::sin(ryaw);

  // Footprint-based collision check rather than a coarse inscribed-radius proxy:
  // inscribed-radius checking caused hard deadlocks where every sample (including v=0)
  // was falsely rejected because the circular proxy overestimated the footprint extent.
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  collision_checker_.setCostmap(costmap);
  const auto & footprint = costmap_ros_->getRobotFootprint();

  double worst_cost = 0.0;

  for (const auto & p : poses_robot_frame) {
    const double wx = rx + p.x * cos_r - p.y * sin_r;
    const double wy = ry + p.x * sin_r + p.y * cos_r;
    const double wtheta = ryaw + p.theta;

    const double cost = collision_checker_.footprintCostAtPose(wx, wy, wtheta, footprint);

    // footprintCostAtPose returns -1 for out-of-costmap poses.
    if (cost < 0.0 || cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)) {
      return -1.0;
    }

    worst_cost = std::max(worst_cost, cost);
  }

  return 1.0 - worst_cost / static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
}

geometry_msgs::msg::Pose2D DWAController::computeLocalGoal(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  geometry_msgs::msg::Pose2D local_goal;

  if (global_plan_.poses.empty()) {
    return local_goal;
  }

  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;
  const double ryaw = tf2::getYaw(robot_pose.pose.orientation);

  size_t closest_idx = 0;
  double closest_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    const double dx = global_plan_.poses[i].pose.position.x - rx;
    const double dy = global_plan_.poses[i].pose.position.y - ry;
    const double d = std::hypot(dx, dy);
    if (d < closest_dist) {
      closest_dist = d;
      closest_idx = i;
    }
  }

  size_t goal_idx = closest_idx;
  double accumulated = 0.0;
  for (size_t i = closest_idx; i + 1 < global_plan_.poses.size(); ++i) {
    const auto & a = global_plan_.poses[i].pose.position;
    const auto & b = global_plan_.poses[i + 1].pose.position;
    accumulated += std::hypot(b.x - a.x, b.y - a.y);
    goal_idx = i + 1;
    if (accumulated >= lookahead_distance_) {
      break;
    }
  }

  // If the lookahead point lands in an inflated/lethal cell, slide goal_idx forward to
  // a clear plan pose. NavFn replans against the global costmap which never sees dynamic
  // obstacles, so the controller must steer around them via this lookahead extension.
  if (obstacle_lookahead_extend_dist_ > 0.0) {
    nav2_costmap_2d::Costmap2D * cm = costmap_ros_->getCostmap();
    double extra_arc = 0.0;
    for (size_t i = goal_idx; i < global_plan_.poses.size(); ++i) {
      unsigned int mx, my;
      const double wx = global_plan_.poses[i].pose.position.x;
      const double wy = global_plan_.poses[i].pose.position.y;
      unsigned char cost = nav2_costmap_2d::FREE_SPACE;
      if (cm->worldToMap(wx, wy, mx, my)) {
        cost = cm->getCost(mx, my);
      }
      if (cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        goal_idx = i;
        break;
      }
      if (i + 1 < global_plan_.poses.size() && extra_arc < obstacle_lookahead_extend_dist_) {
        const auto & a = global_plan_.poses[i].pose.position;
        const auto & b = global_plan_.poses[i + 1].pose.position;
        extra_arc += std::hypot(b.x - a.x, b.y - a.y);
      } else {
        break;
      }
    }
  }

  const bool reached_path_end = (goal_idx + 1 == global_plan_.poses.size());

  const auto & goal_world = global_plan_.poses[goal_idx].pose.position;
  const double dx = goal_world.x - rx;
  const double dy = goal_world.y - ry;

  local_goal.x = dx * std::cos(ryaw) + dy * std::sin(ryaw);
  local_goal.y = -dx * std::sin(ryaw) + dy * std::cos(ryaw);

  // Only switch to the goal's commanded final orientation once within goal_slowdown_distance_
  // (i.e. once the velocity critic has already started braking). Switching earlier causes the
  // robot to curve away from the goal position while still at speed.
  const bool close_to_goal =
    reached_path_end && std::hypot(local_goal.x, local_goal.y) <= goal_slowdown_distance_;

  if (close_to_goal) {
    const double goal_yaw_world = tf2::getYaw(global_plan_.poses[goal_idx].pose.orientation);
    local_goal.theta = angles::shortest_angular_distance(ryaw, goal_yaw_world);
  } else {
    local_goal.theta = std::atan2(local_goal.y, local_goal.x);
  }

  return local_goal;
}

void DWAController::updateOscillationLock(
  const geometry_msgs::msg::Pose2D & robot_pose, double v, double w)
{
  const int v_sign = signOf(v, kSignEps);
  const int w_sign = signOf(w, kSignEps);

  if (x_lock_active_) {
    const double moved = std::hypot(robot_pose.x - x_lock_pose_.x, robot_pose.y - x_lock_pose_.y);
    if (moved >= oscillation_reset_dist_) {
      x_lock_active_ = false;
    }
  }
  if (!x_lock_active_ && v_sign != 0) {
    x_lock_active_ = true;
    x_lock_sign_ = v_sign;
    x_lock_pose_ = robot_pose;
  }

  if (theta_lock_active_) {
    const double turned =
      std::fabs(angles::shortest_angular_distance(theta_lock_pose_.theta, robot_pose.theta));
    if (turned >= oscillation_reset_angle_) {
      theta_lock_active_ = false;
    }
  }
  if (!theta_lock_active_ && w_sign != 0) {
    theta_lock_active_ = true;
    theta_lock_sign_ = w_sign;
    theta_lock_pose_ = robot_pose;
  }
}

bool DWAController::violatesOscillationLock(double v, double w) const
{
  if (x_lock_active_ && signOf(v, kSignEps) == -x_lock_sign_) {
    return true;
  }
  if (theta_lock_active_ && signOf(w, kSignEps) == -theta_lock_sign_) {
    return true;
  }
  return false;
}

void DWAController::scoreTrajectory(
  Trajectory & trajectory,
  const geometry_msgs::msg::Pose2D & local_goal_robot_frame,
  const geometry_msgs::msg::Pose2D & robot_pose) const
{
  const double clearance = trajectoryClearanceCost(trajectory.poses, robot_pose);
  if (clearance < 0.0) {
    trajectory.valid = false;
    trajectory.score = -std::numeric_limits<double>::infinity();
    return;
  }

  const auto & end_pose = trajectory.poses.back();

  const double heading_score =
    1.0 - angleDifference(end_pose.theta, local_goal_robot_frame.theta) / M_PI;

  const double dist_to_local_goal =
    std::hypot(local_goal_robot_frame.x, local_goal_robot_frame.y);
  const double desired_v = goal_slowdown_distance_ > 0.0 ?
    max_vel_x_ * std::clamp(dist_to_local_goal / goal_slowdown_distance_, 0.0, 1.0) :
    max_vel_x_;
  const double velocity_score = max_vel_x_ > 0.0 ?
    1.0 - std::abs(trajectory.v - desired_v) / max_vel_x_ : 0.0;

  trajectory.valid = true;
  trajectory.score =
    heading_weight_ * heading_score +
    clearance_weight_ * clearance +
    velocity_weight_ * velocity_score;
}

geometry_msgs::msg::TwistStamped DWAController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  double v_min, v_max, w_min, w_max;
  computeDynamicWindow(velocity, v_min, v_max, w_min, w_max);

  const geometry_msgs::msg::Pose2D local_goal = computeLocalGoal(pose);

  // Resolve the robot pose once per cycle rather than re-querying TF inside
  // each of the ~200 per-candidate clearance checks.
  geometry_msgs::msg::PoseStamped robot_pose_st;
  if (!costmap_ros_->getRobotPose(robot_pose_st)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "DWAController '%s': could not resolve robot pose in the costmap frame -- stopping",
      plugin_name_.c_str());
    geometry_msgs::msg::TwistStamped stop_cmd;
    stop_cmd.header.frame_id = pose.header.frame_id;
    stop_cmd.header.stamp = clock_->now();
    return stop_cmd;
  }
  geometry_msgs::msg::Pose2D robot_pose_2d;
  robot_pose_2d.x = robot_pose_st.pose.position.x;
  robot_pose_2d.y = robot_pose_st.pose.position.y;
  robot_pose_2d.theta = tf2::getYaw(robot_pose_st.pose.orientation);

  const int n_v = std::max(1, vx_samples_);
  const int n_w = std::max(1, vtheta_samples_);
  const double dv = (n_v > 1) ? (v_max - v_min) / (n_v - 1) : 0.0;
  const double dw = (n_w > 1) ? (w_max - w_min) / (n_w - 1) : 0.0;

  // Evenly spaced grids rarely land exactly on w=0; add it explicitly so
  // "drive straight" is always a candidate (critical in tight corridors).
  std::vector<double> w_samples;
  w_samples.reserve(n_w + 1);
  for (int j = 0; j < n_w; ++j) {
    w_samples.push_back(w_min + j * dw);
  }
  if (w_min <= 0.0 && 0.0 <= w_max) {
    w_samples.push_back(0.0);
  }

  // Two-pass: prefer the top-scoring trajectory that also respects the oscillation lock,
  // but fall back to the global best if nothing does (e.g. a genuine dead-end requiring
  // reversal). The guard can only ever help -- it never turns a solvable cycle into a stop.
  Trajectory best;
  best.score = -std::numeric_limits<double>::infinity();
  Trajectory best_overall;
  best_overall.score = -std::numeric_limits<double>::infinity();

  for (int i = 0; i < n_v; ++i) {
    const double v = v_min + i * dv;
    for (const double w : w_samples) {
      Trajectory candidate;
      candidate.v = v;
      candidate.w = w;
      candidate.poses = simulateTrajectory(v, w);
      scoreTrajectory(candidate, local_goal, robot_pose_2d);

      if (!candidate.valid) {
        continue;
      }
      if (candidate.score > best_overall.score) {
        best_overall = candidate;
      }
      if (!violatesOscillationLock(candidate.v, candidate.w) && candidate.score > best.score) {
        best = candidate;
      }
    }
  }

  if (!best.valid && best_overall.valid) {
    best = best_overall;
  }

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();

  if (!best.valid) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "DWAController '%s': no collision-free trajectory in the dynamic window -- stopping",
      plugin_name_.c_str());
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }

  cmd_vel.twist.linear.x = best.v;
  cmd_vel.twist.angular.z = best.w;

  updateOscillationLock(robot_pose_2d, best.v, best.w);

  nav_msgs::msg::Path local_plan;
  local_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  local_plan.header.stamp = clock_->now();
  local_plan.poses.reserve(best.poses.size());
  for (const auto & p : best.poses) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = local_plan.header;
    ps.pose.position.x = p.x;
    ps.pose.position.y = p.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, p.theta);
    ps.pose.orientation = tf2::toMsg(q);
    local_plan.poses.push_back(ps);
  }
  local_plan_pub_->publish(local_plan);

  return cmd_vel;
}

}  // namespace tb3_dwa_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(tb3_dwa_controller::DWAController, nav2_core::Controller)
