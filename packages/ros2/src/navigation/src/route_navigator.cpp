#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav2_msgs/msg/speed_limit.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class RouteNavigator : public rclcpp::Node {
public:
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using FollowPath = nav2_msgs::action::FollowPath;
  using ComputeGoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

  RouteNavigator() : rclcpp::Node("route_navigator"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    global_path_topic_ = declare_parameter<std::string>("global_path_topic", "/global_path");
    planner_action_ = declare_parameter<std::string>("planner_action", "/compute_path_to_pose");
    planner_id_ = declare_parameter<std::string>("planner_id", "GridBased");
    follow_action_ = declare_parameter<std::string>("follow_action", "/follow_path");
    controller_id_ = declare_parameter<std::string>("controller_id", "FollowPath");
    goal_checker_id_ = declare_parameter<std::string>("goal_checker_id", "general_goal_checker");
    progress_checker_id_ = declare_parameter<std::string>("progress_checker_id", "progress_checker");
    map_frame_ = declare_parameter<std::string>("map_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry/local");
    local_costmap_topic_ = declare_parameter<std::string>("local_costmap_topic", "/local_costmap/costmap_raw");
    speed_limit_topic_ = declare_parameter<std::string>("speed_limit_topic", "/speed_limit");

    go_to_start_ = declare_parameter<bool>("go_to_start", true);
    start_reached_dist_m_ = declare_parameter<double>("start_reached_dist_m", 1.0);
    max_start_goal_distance_m_ = declare_parameter<double>("max_start_goal_distance_m", 35.0);

    goal_ahead_m_ = declare_parameter<double>("goal_ahead_m", 25.0);
    replan_period_s_ = declare_parameter<double>("replan_period_s", 0.5);
    min_follow_replace_period_s_ = declare_parameter<double>("min_follow_replace_period_s", 2.0);

    stuck_time_s_ = declare_parameter<double>("stuck_time_s", 3.0);
    stuck_dist_m_ = declare_parameter<double>("stuck_dist_m", 0.25);

    speed_limit_publish_hz_ = declare_parameter<double>("speed_limit_publish_hz", 10.0);
    speed_limit_max_mps_ = declare_parameter<double>("speed_limit_max_mps", 3.58);
    speed_limit_crawl_mps_ = declare_parameter<double>("speed_limit_crawl_mps", 0.4);
    speed_limit_fallback_mps_ = declare_parameter<double>("speed_limit_fallback_mps", 0.75);
    speed_limit_comfort_decel_mps2_ = declare_parameter<double>("speed_limit_comfort_decel_mps2", 1.0);
    speed_limit_reaction_time_s_ = declare_parameter<double>("speed_limit_reaction_time_s", 1.0);
    speed_limit_buffer_m_ = declare_parameter<double>("speed_limit_buffer_m", 3.0);
    speed_limit_cost_threshold_ = declare_parameter<int>("speed_limit_cost_threshold", 100);
    speed_limit_cap_rise_mps2_ = declare_parameter<double>("speed_limit_cap_rise_mps2", 1.0);
    env_sample_step_m_ = declare_parameter<double>("env_sample_step_m", 0.25);

    path_speed_v_min_mps_ = declare_parameter<double>("path_speed_v_min_mps", 0.2);
    path_speed_a_lat_max_mps2_ = declare_parameter<double>("path_speed_a_lat_max_mps2", 1.0);
    path_speed_k_deadband_ = declare_parameter<double>("path_speed_k_deadband", 0.02);
    path_speed_preview_min_m_ = declare_parameter<double>("path_speed_preview_min_m", 12.0);
    path_speed_preview_max_m_ = declare_parameter<double>("path_speed_preview_max_m", 30.0);

    go_to_start_speed_cap_mps_ = declare_parameter<double>("go_to_start_speed_cap_mps", 2.0);
    start_approach_window_m_ = declare_parameter<double>("start_approach_window_m", 8.0);
    start_approach_speed_cap_mps_ = declare_parameter<double>("start_approach_speed_cap_mps", 1.0);
    start_final_window_m_ = declare_parameter<double>("start_final_window_m", 3.0);
    start_final_speed_cap_mps_ = declare_parameter<double>("start_final_speed_cap_mps", 0.6);
    goal_approach_window_m_ = declare_parameter<double>("goal_approach_window_m", 10.0);
    goal_approach_speed_cap_mps_ = declare_parameter<double>("goal_approach_speed_cap_mps", 1.2);
    goal_final_window_m_ = declare_parameter<double>("goal_final_window_m", 4.0);
    goal_final_speed_cap_mps_ = declare_parameter<double>("goal_final_speed_cap_mps", 0.6);

    planner_client_ = rclcpp_action::create_client<ComputePathToPose>(this, planner_action_);
    follow_client_ = rclcpp_action::create_client<FollowPath>(this, follow_action_);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      global_path_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&RouteNavigator::onGlobalPath, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      std::bind(&RouteNavigator::onOdom, this, std::placeholders::_1));

    local_costmap_sub_ = create_subscription<nav2_msgs::msg::Costmap>(
      local_costmap_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&RouteNavigator::onLocalCostmap, this, std::placeholders::_1));

    planned_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/route_navigator/planned_path", rclcpp::QoS(1).transient_local().reliable());

    speed_limit_pub_ = create_publisher<nav2_msgs::msg::SpeedLimit>(
      speed_limit_topic_, rclcpp::QoS(1).transient_local().reliable());

    const auto plan_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(replan_period_s_));
    timer_ = create_wall_timer(plan_period, std::bind(&RouteNavigator::tick, this));

    const auto speed_limit_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(1e-3, speed_limit_publish_hz_)));
    speed_limit_timer_ = create_wall_timer(speed_limit_period, std::bind(&RouteNavigator::onSpeedLimitTimer, this));

    last_pose_time_ = now();
    last_pose_.pose.orientation.w = 1.0;
    last_local_costmap_time_ = now();
    last_speed_limit_eval_time_ = now();
    last_follow_send_time_ = now();
    publishSpeedLimitNoLimit();
  }

private:
  enum class Mode { GO_TO_START, FOLLOW_ROUTE };

  template <typename GoalT>
  static auto setProgressCheckerIdIfPresent(GoalT &goal, const std::string &id) -> decltype(goal.progress_checker_id = id, void()) {
    if (!id.empty()) goal.progress_checker_id = id;
  }

  template <typename GoalT> static void setProgressCheckerIdIfPresent(GoalT &, const std::string &) {}

  static double clampDouble(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
  }

  static double dist2(double ax, double ay, double bx, double by) {
    const double dx = ax - bx;
    const double dy = ay - by;
    return dx * dx + dy * dy;
  }

  static double dist(const geometry_msgs::msg::PoseStamped &a, const geometry_msgs::msg::PoseStamped &b) {
    return std::sqrt(dist2(
      a.pose.position.x, a.pose.position.y,
      b.pose.position.x, b.pose.position.y));
  }

  static geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(q);
  }

  static double signedCurvature(
    const geometry_msgs::msg::PoseStamped &a,
    const geometry_msgs::msg::PoseStamped &b,
    const geometry_msgs::msg::PoseStamped &c)
  {
    const double abx = b.pose.position.x - a.pose.position.x;
    const double aby = b.pose.position.y - a.pose.position.y;
    const double bcx = c.pose.position.x - b.pose.position.x;
    const double bcy = c.pose.position.y - b.pose.position.y;
    const double acx = c.pose.position.x - a.pose.position.x;
    const double acy = c.pose.position.y - a.pose.position.y;

    const double ab = std::sqrt(abx * abx + aby * aby);
    const double bc = std::sqrt(bcx * bcx + bcy * bcy);
    const double ac = std::sqrt(acx * acx + acy * acy);
    if (ab < 1e-6 || bc < 1e-6 || ac < 1e-6) return 0.0;

    const double cross = abx * bcy - aby * bcx;
    const double area2 = std::abs(cross);
    const double k = (2.0 * area2) / (ab * bc * ac);
    return (cross >= 0.0) ? k : -k;
  }

  static double speedFromCurvature(double k, double vMin, double vMax, double aLatMax, double kDeadband) {
    const double ak = std::abs(k);
    if (ak < kDeadband) return vMax;
    const double v = std::sqrt(std::max(0.0, aLatMax / ak));
    return clampDouble(v, vMin, vMax);
  }

  static std::string makePathKey(const nav_msgs::msg::Path &path, double resolution_m) {
    const double scale = (resolution_m > 1e-9) ? (1.0 / resolution_m) : 1.0;
    std::string key;
    key.reserve(path.poses.size() * 24 + 16);
    key += std::to_string(path.poses.size());
    key.push_back(':');

    for (const auto &pose : path.poses) {
      key += std::to_string(std::llround(pose.pose.position.x * scale));
      key.push_back(',');
      key += std::to_string(std::llround(pose.pose.position.y * scale));
      key.push_back(';');
    }

    return key;
  }

  void publishEmptyPlannedPath() {
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = now();
    planned_path_pub_->publish(path);
  }

  void publishSpeedLimitValue(double speed_limit) {
    nav2_msgs::msg::SpeedLimit msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.percentage = false;
    msg.speed_limit = speed_limit;
    speed_limit_pub_->publish(msg);
  }

  void publishSpeedLimitNoLimit() {
    publishSpeedLimitValue(0.0);
  }

  void clearActiveLocalPlan() {
    have_local_plan_ = false;
    latest_local_plan_ = nav_msgs::msg::Path();
  }

  void setActiveLocalPlan(const nav_msgs::msg::Path &path) {
    latest_local_plan_ = path;
    have_local_plan_ = latest_local_plan_.poses.size() >= 2;
  }

  void resetSpeedLimitState() {
    have_last_speed_limit_cap_ = false;
    last_speed_limit_cap_mps_ = speed_limit_max_mps_;
    last_speed_limit_eval_time_ = now();
  }

  void clearRoute() {
    route_version_++;
    active_plan_token_++;
    active_follow_token_++;

    have_path_ = false;
    planning_ = false;
    stuck_ = false;
    global_path_ = nav_msgs::msg::Path();
    clearActiveLocalPlan();
    monotonic_i_ = 0;
    nearest_i_ = 0;
    active_follow_path_key_.clear();
    mode_ = go_to_start_ ? Mode::GO_TO_START : Mode::FOLLOW_ROUTE;

    planner_client_->async_cancel_all_goals();
    follow_client_->async_cancel_all_goals();
    active_follow_.reset();
    publishEmptyPlannedPath();
    resetSpeedLimitState();
    publishSpeedLimitNoLimit();
  }

  void onGlobalPath(const nav_msgs::msg::Path::SharedPtr msg) {
    if (!msg) return;
    if (msg->poses.empty()) {
      clearRoute();
      return;
    }
    if (msg->poses.size() < 2) return;

    route_version_++;
    active_plan_token_++;
    planning_ = false;
    planner_client_->async_cancel_all_goals();

    global_path_ = *msg;
    have_path_ = true;
    clearActiveLocalPlan();

    if (go_to_start_) {
      mode_ = Mode::GO_TO_START;
    } else {
      mode_ = Mode::FOLLOW_ROUTE;
      nearest_i_ = 0;
    }

    monotonic_i_ = 0;
    resetSpeedLimitState();
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!msg) return;
    odom_ = *msg;
    have_odom_ = true;
  }

  void onLocalCostmap(const nav2_msgs::msg::Costmap::SharedPtr msg) {
    if (!msg) return;
    local_costmap_ = *msg;
    have_local_costmap_ = true;
    last_local_costmap_time_ = now();
  }

  bool getRobotPose(geometry_msgs::msg::PoseStamped &out) {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return false;
    }

    out.header.frame_id = map_frame_;
    out.header.stamp = now();
    out.pose.position.x = tf.transform.translation.x;
    out.pose.position.y = tf.transform.translation.y;
    out.pose.position.z = tf.transform.translation.z;
    out.pose.orientation = tf.transform.rotation;
    return true;
  }

  size_t findNearestIndexForwardOnly(const nav_msgs::msg::Path &p, const geometry_msgs::msg::PoseStamped &pose, size_t from) {
    const size_t n = p.poses.size();
    if (n == 0) return 0;
    if (from >= n - 1) return n - 1;

    const double rx = pose.pose.position.x;
    const double ry = pose.pose.position.y;

    const size_t window = 600;
    const size_t start = from;
    const size_t end = std::min(n - 1, from + window);

    size_t best = start;
    double bestd = dist2(rx, ry, p.poses[start].pose.position.x, p.poses[start].pose.position.y);

    for (size_t i = start + 1; i <= end; i++) {
      const double d = dist2(rx, ry, p.poses[i].pose.position.x, p.poses[i].pose.position.y);
      if (d < bestd) {
        bestd = d;
        best = i;
      }
    }

    return best;
  }

  size_t advanceByDistance(const nav_msgs::msg::Path &p, size_t from, double ahead_m) {
    const size_t n = p.poses.size();
    if (n == 0) return 0;
    if (from >= n - 1) return n - 1;

    double acc = 0.0;
    size_t i = from;

    while (i + 1 < n && acc < ahead_m) {
      acc += dist(p.poses[i], p.poses[i + 1]);
      i++;
    }

    return i;
  }

  void updateStuck(const geometry_msgs::msg::PoseStamped &pose) {
    const auto t = now();
    const double d = std::sqrt(dist2(
      pose.pose.position.x, pose.pose.position.y,
      last_pose_.pose.position.x, last_pose_.pose.position.y));

    if ((t - last_pose_time_).seconds() >= stuck_time_s_) {
      stuck_ = (d < stuck_dist_m_);
      last_pose_time_ = t;
      last_pose_ = pose;
    }
  }

  double distanceToPathEnd(const nav_msgs::msg::Path &path, size_t from, const geometry_msgs::msg::PoseStamped &pose) const {
    if (path.poses.empty()) return 0.0;
    const size_t start = std::min(from, path.poses.size() - 1);
    double remaining = dist(pose, path.poses[start]);
    for (size_t i = start; i + 1 < path.poses.size(); i++) {
      remaining += dist(path.poses[i], path.poses[i + 1]);
    }
    return remaining;
  }

  double computePathPreviewDistance(double v_meas) const {
    const double preview = v_meas * speed_limit_reaction_time_s_ +
      (v_meas * v_meas) / (2.0 * std::max(1e-6, speed_limit_comfort_decel_mps2_)) + 4.0;
    return clampDouble(preview, path_speed_preview_min_m_, path_speed_preview_max_m_);
  }

  double computePathSpeedCap(size_t route_index, double preview_m) const {
    const size_t n = global_path_.poses.size();
    if (n < 3) return speed_limit_max_mps_;

    const size_t start = std::min(route_index, n - 1);
    size_t end = start;
    double acc = 0.0;

    while (end + 1 < n && acc < preview_m) {
      acc += dist(global_path_.poses[end], global_path_.poses[end + 1]);
      end++;
    }

    if (end <= start + 1) return speed_limit_max_mps_;

    const size_t i_begin = std::max<size_t>(1, start);
    const size_t i_end = std::min(end, n - 2);
    if (i_begin > i_end) return speed_limit_max_mps_;

    double cap = speed_limit_max_mps_;
    for (size_t i = i_begin; i <= i_end; i++) {
      const double k = signedCurvature(global_path_.poses[i - 1], global_path_.poses[i], global_path_.poses[i + 1]);
      cap = std::min(cap, speedFromCurvature(
        k,
        path_speed_v_min_mps_,
        speed_limit_max_mps_,
        path_speed_a_lat_max_mps2_,
        path_speed_k_deadband_));
    }

    return cap;
  }

  double computePhaseCap(const geometry_msgs::msg::PoseStamped &robot_pose, size_t route_index) const {
    if (mode_ == Mode::GO_TO_START) {
      double cap = go_to_start_speed_cap_mps_;
      const double dist_to_start = dist(robot_pose, global_path_.poses.front());
      if (dist_to_start <= start_approach_window_m_) cap = std::min(cap, start_approach_speed_cap_mps_);
      if (dist_to_start <= start_final_window_m_) cap = std::min(cap, start_final_speed_cap_mps_);
      return cap;
    }

    double cap = speed_limit_max_mps_;
    const double remaining = distanceToPathEnd(global_path_, route_index, robot_pose);
    if (remaining <= goal_approach_window_m_) cap = std::min(cap, goal_approach_speed_cap_mps_);
    if (remaining <= goal_final_window_m_) cap = std::min(cap, goal_final_speed_cap_mps_);
    return cap;
  }

  bool worldToGrid(const nav2_msgs::msg::Costmap &costmap, double wx, double wy, size_t &gx, size_t &gy) const {
    const double resolution = static_cast<double>(costmap.metadata.resolution);
    if (resolution <= 0.0 || costmap.metadata.size_x == 0 || costmap.metadata.size_y == 0) return false;

    const auto &origin = costmap.metadata.origin;
    tf2::Quaternion q;
    tf2::fromMsg(origin.orientation, q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    const double dx = wx - origin.position.x;
    const double dy = wy - origin.position.y;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double lx = c * dx + s * dy;
    const double ly = -s * dx + c * dy;

    if (lx < 0.0 || ly < 0.0) return false;

    const auto gx_i = static_cast<size_t>(std::floor(lx / resolution));
    const auto gy_i = static_cast<size_t>(std::floor(ly / resolution));
    if (gx_i >= costmap.metadata.size_x || gy_i >= costmap.metadata.size_y) return false;

    gx = gx_i;
    gy = gy_i;
    return true;
  }

  double computeFreeDistanceOnPlan(const nav_msgs::msg::Path &path) const {
    if (!have_local_costmap_ || path.poses.size() < 2) return std::numeric_limits<double>::infinity();

    double traveled = 0.0;
    double next_sample = 0.0;
    const double step = std::max(1e-3, env_sample_step_m_);
    const auto &costmap = local_costmap_;

    for (size_t i = 0; i + 1 < path.poses.size(); i++) {
      const auto &a = path.poses[i].pose.position;
      const auto &b = path.poses[i + 1].pose.position;
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double seg_len = std::sqrt(dx * dx + dy * dy);
      if (seg_len < 1e-9) continue;

      while (next_sample <= traveled + seg_len + 1e-9) {
        const double local_s = clampDouble(next_sample - traveled, 0.0, seg_len);
        const double ratio = local_s / seg_len;
        const double wx = a.x + dx * ratio;
        const double wy = a.y + dy * ratio;

        size_t gx = 0;
        size_t gy = 0;
        if (!worldToGrid(costmap, wx, wy, gx, gy)) return next_sample;

        const size_t idx = gy * costmap.metadata.size_x + gx;
        if (idx >= costmap.data.size()) return next_sample;
        if (static_cast<int>(costmap.data[idx]) >= speed_limit_cost_threshold_) return next_sample;

        next_sample += step;
      }

      traveled += seg_len;
    }

    return std::numeric_limits<double>::infinity();
  }

  double computeEnvironmentSpeedCap() const {
    const double free_distance = computeFreeDistanceOnPlan(latest_local_plan_);
    if (!std::isfinite(free_distance)) return speed_limit_max_mps_;

    const double a = std::max(1e-6, speed_limit_comfort_decel_mps2_);
    const double reaction_term = a * std::max(0.0, speed_limit_reaction_time_s_);
    const double L = std::max(0.0, free_distance - speed_limit_buffer_m_);
    const double v = -reaction_term + std::sqrt(reaction_term * reaction_term + 2.0 * a * L);
    return clampDouble(v, 0.0, speed_limit_max_mps_);
  }

  double rateLimitSpeedCapIncrease(double target) {
    const auto t = now();
    const double dt = std::max(0.0, (t - last_speed_limit_eval_time_).seconds());
    last_speed_limit_eval_time_ = t;

    if (!have_last_speed_limit_cap_) {
      have_last_speed_limit_cap_ = true;
      last_speed_limit_cap_mps_ = target;
      return target;
    }

    if (target > last_speed_limit_cap_mps_) {
      target = std::min(target, last_speed_limit_cap_mps_ + speed_limit_cap_rise_mps2_ * dt);
    }

    last_speed_limit_cap_mps_ = target;
    return target;
  }

  void onSpeedLimitTimer() {
    if (!have_path_) {
      resetSpeedLimitState();
      publishSpeedLimitNoLimit();
      return;
    }

    if (!have_odom_) {
      const double final_cap = rateLimitSpeedCapIncrease(speed_limit_fallback_mps_);
      publishSpeedLimitValue(final_cap);
      return;
    }

    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose)) {
      const double final_cap = rateLimitSpeedCapIncrease(speed_limit_fallback_mps_);
      publishSpeedLimitValue(final_cap);
      return;
    }

    if (!have_local_costmap_ || (now() - last_local_costmap_time_).seconds() > 0.5) {
      const double final_cap = rateLimitSpeedCapIncrease(speed_limit_fallback_mps_);
      publishSpeedLimitValue(final_cap);
      return;
    }

    size_t route_index = 0;
    if (mode_ == Mode::FOLLOW_ROUTE && !global_path_.poses.empty()) {
      route_index = findNearestIndexForwardOnly(global_path_, robot_pose, monotonic_i_);
      monotonic_i_ = std::max(monotonic_i_, route_index);
    }

    const double v_meas = std::abs(odom_.twist.twist.linear.x);
    const double phase_cap = computePhaseCap(robot_pose, route_index);
    const double path_cap = (mode_ == Mode::FOLLOW_ROUTE)
      ? computePathSpeedCap(route_index, computePathPreviewDistance(v_meas))
      : speed_limit_max_mps_;

    const bool have_env_cap = have_local_plan_;
    const double env_cap = have_env_cap ? computeEnvironmentSpeedCap() : speed_limit_max_mps_;

    double target_cap = std::min(speed_limit_max_mps_, phase_cap);
    if (mode_ == Mode::FOLLOW_ROUTE) target_cap = std::min(target_cap, path_cap);
    if (have_env_cap) {
      const double env_cap_with_floor =
        (env_cap > 0.0 && env_cap < speed_limit_crawl_mps_) ? speed_limit_crawl_mps_ : env_cap;
      target_cap = std::min(target_cap, env_cap_with_floor);
    }

    const double final_cap = rateLimitSpeedCapIncrease(target_cap);
    publishSpeedLimitValue(final_cap);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "speed cap path=%.2f phase=%.2f env=%.2f final=%.2f",
      path_cap, phase_cap, env_cap, final_cap);
  }

  void tick() {
    if (!have_path_) return;
    if (planning_) return;

    geometry_msgs::msg::PoseStamped start;
    if (!getRobotPose(start)) return;

    updateStuck(start);

    if (!planner_client_->wait_for_action_server(std::chrono::milliseconds(50))) return;
    if (!follow_client_->wait_for_action_server(std::chrono::milliseconds(50))) return;

    geometry_msgs::msg::PoseStamped goal;

    if (mode_ == Mode::GO_TO_START) {
      goal = global_path_.poses.front();
      goal.header.frame_id = map_frame_;
      goal.header.stamp = start.header.stamp;

      const double dist_to_start = dist(start, goal);
      if (dist_to_start <= start_reached_dist_m_) {
        mode_ = Mode::FOLLOW_ROUTE;
        monotonic_i_ = 0;
      } else if (max_start_goal_distance_m_ > 1e-6 && dist_to_start > max_start_goal_distance_m_) {
        const double dx = goal.pose.position.x - start.pose.position.x;
        const double dy = goal.pose.position.y - start.pose.position.y;
        const double scale = max_start_goal_distance_m_ / dist_to_start;
        goal.pose.position.x = start.pose.position.x + dx * scale;
        goal.pose.position.y = start.pose.position.y + dy * scale;
        goal.pose.position.z = start.pose.position.z;
        goal.pose.orientation = yawToQuaternion(std::atan2(dy, dx));
      }
    }

    if (mode_ == Mode::FOLLOW_ROUTE) {
      if (monotonic_i_ >= global_path_.poses.size() - 1) return;

      const size_t nearest = findNearestIndexForwardOnly(global_path_, start, monotonic_i_);
      monotonic_i_ = std::max(monotonic_i_, nearest);

      const double ahead = stuck_ ? (goal_ahead_m_ * 1.5) : goal_ahead_m_;
      const size_t goal_i = advanceByDistance(global_path_, monotonic_i_, ahead);

      goal = global_path_.poses[goal_i];
      goal.header.frame_id = map_frame_;
      goal.header.stamp = start.header.stamp;
    }

    ComputePathToPose::Goal cg;
    cg.start = start;
    cg.goal = goal;
    cg.use_start = true;
    cg.planner_id = planner_id_;

    const uint64_t route_version = route_version_;
    const uint64_t plan_token = ++active_plan_token_;
    planning_ = true;

    rclcpp_action::Client<ComputePathToPose>::SendGoalOptions opts;
    opts.result_callback = [this, route_version, plan_token](const ComputeGoalHandle::WrappedResult &res) {
      if (plan_token != active_plan_token_) return;
      planning_ = false;
      if (route_version != route_version_) return;
      if (res.code != rclcpp_action::ResultCode::SUCCEEDED || !res.result) {
        if (!active_follow_) clearActiveLocalPlan();
        return;
      }

      const auto &path = res.result->path;
      if (path.poses.size() < 2) {
        if (!active_follow_) clearActiveLocalPlan();
        return;
      }

      planned_path_pub_->publish(path);

      const std::string path_key = makePathKey(path, 0.25);

      if (active_follow_) {
        const double since_last_follow_send = (now() - last_follow_send_time_).seconds();
        if (path_key == active_follow_path_key_ || since_last_follow_send < min_follow_replace_period_s_) {
          return;
        }
      }

      sendFollow(path, path_key);
    };

    planner_client_->async_send_goal(cg, opts);
  }

  void sendFollow(const nav_msgs::msg::Path &path, const std::string &path_key) {
    const uint64_t follow_token = ++active_follow_token_;
    if (active_follow_) {
      follow_client_->async_cancel_all_goals();
    }
    last_follow_send_time_ = now();
    setActiveLocalPlan(path);

    FollowPath::Goal fg;
    fg.path = path;
    fg.controller_id = controller_id_;
    fg.goal_checker_id = goal_checker_id_;
    setProgressCheckerIdIfPresent(fg, progress_checker_id_);

    rclcpp_action::Client<FollowPath>::SendGoalOptions opts;

    opts.goal_response_callback = [this, follow_token, path_key](const FollowGoalHandle::SharedPtr &h) {
      if (follow_token != active_follow_token_) return;
      if (!h) {
        active_follow_.reset();
        active_follow_path_key_.clear();
        clearActiveLocalPlan();
        return;
      }
      active_follow_ = h;
      active_follow_path_key_ = path_key;
    };

    opts.result_callback = [this, follow_token](const FollowGoalHandle::WrappedResult &) {
      if (follow_token != active_follow_token_) return;
      active_follow_.reset();
      active_follow_path_key_.clear();
      clearActiveLocalPlan();
    };

    follow_client_->async_send_goal(fg, opts);
  }

  std::string global_path_topic_;
  std::string planner_action_;
  std::string planner_id_;
  std::string follow_action_;
  std::string controller_id_;
  std::string goal_checker_id_;
  std::string progress_checker_id_;
  std::string map_frame_;
  std::string base_frame_;
  std::string odom_topic_;
  std::string local_costmap_topic_;
  std::string speed_limit_topic_;

  bool go_to_start_{true};
  double start_reached_dist_m_{1.0};
  double max_start_goal_distance_m_{35.0};

  double goal_ahead_m_{25.0};
  double replan_period_s_{0.5};
  double min_follow_replace_period_s_{2.0};
  double stuck_time_s_{3.0};
  double stuck_dist_m_{0.25};

  double speed_limit_publish_hz_{10.0};
  double speed_limit_max_mps_{3.58};
  double speed_limit_crawl_mps_{0.4};
  double speed_limit_fallback_mps_{0.75};
  double speed_limit_comfort_decel_mps2_{1.0};
  double speed_limit_reaction_time_s_{1.0};
  double speed_limit_buffer_m_{3.0};
  int speed_limit_cost_threshold_{100};
  double speed_limit_cap_rise_mps2_{1.0};
  double env_sample_step_m_{0.25};

  double path_speed_v_min_mps_{0.2};
  double path_speed_a_lat_max_mps2_{1.0};
  double path_speed_k_deadband_{0.02};
  double path_speed_preview_min_m_{12.0};
  double path_speed_preview_max_m_{30.0};

  double go_to_start_speed_cap_mps_{2.0};
  double start_approach_window_m_{8.0};
  double start_approach_speed_cap_mps_{1.0};
  double start_final_window_m_{3.0};
  double start_final_speed_cap_mps_{0.6};
  double goal_approach_window_m_{10.0};
  double goal_approach_speed_cap_mps_{1.2};
  double goal_final_window_m_{4.0};
  double goal_final_speed_cap_mps_{0.6};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp_action::Client<ComputePathToPose>::SharedPtr planner_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr local_costmap_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr speed_limit_timer_;

  nav_msgs::msg::Path global_path_;
  nav_msgs::msg::Path latest_local_plan_;
  nav_msgs::msg::Odometry odom_;
  nav2_msgs::msg::Costmap local_costmap_;
  bool have_path_{false};
  bool have_local_plan_{false};
  bool have_odom_{false};
  bool have_local_costmap_{false};
  bool planning_{false};
  bool stuck_{false};

  Mode mode_{Mode::GO_TO_START};

  size_t nearest_i_{0};
  size_t monotonic_i_{0};
  FollowGoalHandle::SharedPtr active_follow_;
  std::string active_follow_path_key_;
  uint64_t route_version_{0};
  uint64_t active_plan_token_{0};
  uint64_t active_follow_token_{0};

  bool have_last_speed_limit_cap_{false};
  double last_speed_limit_cap_mps_{3.58};

  rclcpp::Time last_pose_time_;
  rclcpp::Time last_local_costmap_time_;
  rclcpp::Time last_speed_limit_eval_time_;
  rclcpp::Time last_follow_send_time_;
  geometry_msgs::msg::PoseStamped last_pose_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RouteNavigator>());
  rclcpp::shutdown();
  return 0;
}
