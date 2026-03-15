#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
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
    map_frame_ = declare_parameter<std::string>("map_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    go_to_start_ = declare_parameter<bool>("go_to_start", true);
    start_reached_dist_m_ = declare_parameter<double>("start_reached_dist_m", 1.0);

    goal_ahead_m_ = declare_parameter<double>("goal_ahead_m", 25.0);
    replan_period_s_ = declare_parameter<double>("replan_period_s", 0.5);

    stuck_time_s_ = declare_parameter<double>("stuck_time_s", 3.0);
    stuck_dist_m_ = declare_parameter<double>("stuck_dist_m", 0.25);

    planner_client_ = rclcpp_action::create_client<ComputePathToPose>(this, planner_action_);
    follow_client_ = rclcpp_action::create_client<FollowPath>(this, follow_action_);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      global_path_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&RouteNavigator::onGlobalPath, this, std::placeholders::_1));

    planned_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/route_navigator/planned_path", rclcpp::QoS(1).transient_local().reliable());

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(replan_period_s_));
    timer_ = create_wall_timer(period, std::bind(&RouteNavigator::tick, this));

    last_pose_time_ = now();
    last_pose_.pose.orientation.w = 1.0;
  }

private:
  enum class Mode { GO_TO_START, FOLLOW_ROUTE };

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

  void onGlobalPath(const nav_msgs::msg::Path::SharedPtr msg) {
    if (!msg) return;
    if (msg->poses.size() < 2) return;

    global_path_ = *msg;
    have_path_ = true;

    if (go_to_start_) {
      mode_ = Mode::GO_TO_START;
    } else {
      mode_ = Mode::FOLLOW_ROUTE;
      nearest_i_ = 0;
    }

    monotonic_i_ = 0;
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
      const double x0 = p.poses[i].pose.position.x;
      const double y0 = p.poses[i].pose.position.y;
      const double x1 = p.poses[i + 1].pose.position.x;
      const double y1 = p.poses[i + 1].pose.position.y;
      acc += std::sqrt(dist2(x0, y0, x1, y1));
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

      if (dist(start, goal) <= start_reached_dist_m_) {
        mode_ = Mode::FOLLOW_ROUTE;
        monotonic_i_ = 0;
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

    planning_ = true;

    rclcpp_action::Client<ComputePathToPose>::SendGoalOptions opts;
    opts.result_callback = [this](const ComputeGoalHandle::WrappedResult &res) {
      planning_ = false;
      if (res.code != rclcpp_action::ResultCode::SUCCEEDED) return;
      const auto &path = res.result->path;
      if (path.poses.size() < 2) return;
      planned_path_pub_->publish(path);
      sendFollow(path);
    };

    planner_client_->async_send_goal(cg, opts);
  }

  void sendFollow(const nav_msgs::msg::Path &path) {
    follow_client_->async_cancel_all_goals();

    FollowPath::Goal fg;
    fg.path = path;
    fg.controller_id = controller_id_;
    fg.goal_checker_id = goal_checker_id_;

    rclcpp_action::Client<FollowPath>::SendGoalOptions opts;

    opts.goal_response_callback = [this](const FollowGoalHandle::SharedPtr &h) {
      active_follow_ = h;
    };

    opts.result_callback = [this](const FollowGoalHandle::WrappedResult &) {
      active_follow_.reset();
    };

    follow_client_->async_send_goal(fg, opts);
  }

  std::string global_path_topic_;
  std::string planner_action_;
  std::string planner_id_;
  std::string follow_action_;
  std::string controller_id_;
  std::string goal_checker_id_;
  std::string map_frame_;
  std::string base_frame_;

  bool go_to_start_{true};
  double start_reached_dist_m_{1.0};

  double goal_ahead_m_{25.0};
  double replan_period_s_{0.5};
  double stuck_time_s_{3.0};
  double stuck_dist_m_{0.25};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp_action::Client<ComputePathToPose>::SharedPtr planner_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path global_path_;
  bool have_path_{false};
  bool planning_{false};
  bool stuck_{false};

  Mode mode_{Mode::GO_TO_START};

  size_t nearest_i_{0};
  size_t monotonic_i_{0};
  FollowGoalHandle::SharedPtr active_follow_;

  rclcpp::Time last_pose_time_;
  geometry_msgs::msg::PoseStamped last_pose_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RouteNavigator>());
  rclcpp::shutdown();
  return 0;
}
