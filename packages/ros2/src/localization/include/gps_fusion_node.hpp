#pragma once

#include <memory>
#include <optional>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class GpsFusionNode : public rclcpp::Node {
public:
  GpsFusionNode();

private:
  double speed_threshold_;
  double gps_pos_variance_;
  double yaw_variance_;

  std::optional<double> last_x_;
  std::optional<double> last_y_;
  std::optional<double> last_confident_yaw_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fastlio_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void gpsCallback(nav_msgs::msg::Odometry::SharedPtr msg);
  void localCallback(nav_msgs::msg::Odometry::SharedPtr msg);
  void maybePublish(const rclcpp::Time & stamp);
};
