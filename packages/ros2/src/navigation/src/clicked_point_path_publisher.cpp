#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class ClickedPointPathPublisher : public rclcpp::Node {
public:
  ClickedPointPathPublisher() : rclcpp::Node("clicked_point_path_publisher"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    global_frame_ = this->declare_parameter<std::string>("global_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    clicked_topic_ = this->declare_parameter<std::string>("clicked_topic", "/clicked_point");
    path_topic_ = this->declare_parameter<std::string>("path_topic", "/global_path");
    spacing_m_ = this->declare_parameter<double>("spacing_m", 0.75);
    tf_timeout_s_ = this->declare_parameter<double>("tf_timeout_s", 0.2);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, qos);

    clicked_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        clicked_topic_, 10, std::bind(&ClickedPointPathPublisher::onClickedPoint, this, std::placeholders::_1));
  }

private:
  void onClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    if (!msg) return;

    geometry_msgs::msg::PointStamped goal_global;

    try {
      if (msg->header.frame_id == global_frame_) {
        goal_global = *msg;
      } else {
        goal_global = tf_buffer_.transform(*msg, global_frame_, tf2::durationFromSec(tf_timeout_s_));
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "clicked point transform failed: %s", e.what());
      return;
    }

    geometry_msgs::msg::TransformStamped tf_ob;
    try {
      tf_ob = tf_buffer_.lookupTransform(global_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_s_));
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "robot pose tf lookup failed: %s", e.what());
      return;
    }

    const double rx = tf_ob.transform.translation.x;
    const double ry = tf_ob.transform.translation.y;

    const double gx = goal_global.point.x;
    const double gy = goal_global.point.y;

    const double dx = gx - rx;
    const double dy = gy - ry;
    const double dist = std::sqrt(dx * dx + dy * dy);
    if (!std::isfinite(dist) || dist < 0.2) return;

    const double spacing = std::max(0.05, spacing_m_);
    const int n = std::max(2, static_cast<int>(std::ceil(dist / spacing)) + 1);

    const double yaw = std::atan2(dy, dx);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    const auto q_msg = tf2::toMsg(q);

    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = global_frame_;
    path.poses.reserve(static_cast<size_t>(n));

    for (int i = 0; i < n; i++) {
      const double t = (n == 1) ? 0.0 : (static_cast<double>(i) / static_cast<double>(n - 1));
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = rx + dx * t;
      ps.pose.position.y = ry + dy * t;
      ps.pose.position.z = 0.0;
      ps.pose.orientation = q_msg;
      path.poses.push_back(ps);
    }

    path_pub_->publish(path);
  }

  std::string global_frame_;
  std::string base_frame_;
  std::string clicked_topic_;
  std::string path_topic_;
  double spacing_m_{0.75};
  double tf_timeout_s_{0.2};

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClickedPointPathPublisher>());
  rclcpp::shutdown();
  return 0;
}
