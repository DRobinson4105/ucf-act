#include "gps_fusion_node.hpp"

#include <cmath>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

GpsFusionNode::GpsFusionNode()
: rclcpp::Node("gps_fusion_node")
{
  declare_parameter("speed_threshold",  0.3);
  declare_parameter("gps_pos_variance", 1.0);
  declare_parameter("yaw_variance",     0.05);

  speed_threshold_  = get_parameter("speed_threshold").as_double();
  gps_pos_variance_ = get_parameter("gps_pos_variance").as_double();
  yaw_variance_     = get_parameter("yaw_variance").as_double();

  tf_buffer_      = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/global", 10);

  gps_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/gps", rclcpp::SensorDataQoS(),
    std::bind(&GpsFusionNode::gpsCallback, this, std::placeholders::_1));

  // Use FAST_LIO raw odometry - /Odometry carries real velocity.
  fastlio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/Odometry", rclcpp::SensorDataQoS(),
    std::bind(&GpsFusionNode::localCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "GPS fusion node started (speed_threshold=%.2f m/s, gps_pos_variance=%.3f m², yaw_variance=%.3f rad²)",
    speed_threshold_, gps_pos_variance_, yaw_variance_);
}

// - GPS callback 
// navsat_transform already converted lat/lon -> local Cartesian (same frame as
// /odometry/local), so we use x, y directly.

void GpsFusionNode::gpsCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_x_ = msg->pose.pose.position.x;
  last_y_ = msg->pose.pose.position.y;
  maybePublish(msg->header.stamp);
}

// FAST_LIO odometry callback 
// Speed from FAST_LIO twist
// Yaw from FAST_LIO pose orientation (camera_init frame = odom frame per static TF).
// Only accept yaw when the cart is moving - stationary yaw drifts.

void GpsFusionNode::localCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  const double vx = msg->twist.twist.linear.x;
  const double vy = msg->twist.twist.linear.y;
  const double speed = std::sqrt(vx * vx + vy * vy);

  if (speed > speed_threshold_) {
    last_confident_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
  }
}

// ── Publish 
// Fuse: position from GPS, yaw from local odometry (when moving).
// Derive map→odom TF from the fused map -> base_link and ekf_local's odom -> base_link.

void GpsFusionNode::maybePublish(const rclcpp::Time & stamp)
{
  if (!last_x_ || !last_y_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for GPS fix");
    return;
  }
  if (!last_confident_yaw_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Waiting for confident yaw — drive the cart above %.2f m/s", speed_threshold_);
    return;
  }

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, *last_confident_yaw_);

  // T_map_base: where base_link is in the map frame (GPS + yaw)
  tf2::Transform T_map_base;
  T_map_base.setOrigin({*last_x_, *last_y_, 0.0});
  T_map_base.setRotation(q);

  // T_base_odom: base_link -> odom (inverse of ekf_local's odom -> base_link TF)
  geometry_msgs::msg::TransformStamped T_base_odom_msg;
  try {
    T_base_odom_msg = tf_buffer_->lookupTransform("base_link", "odom", tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Cannot look up base_link->odom: %s", e.what());
    return;
  }

  tf2::Transform T_base_odom;
  tf2::fromMsg(T_base_odom_msg.transform, T_base_odom);

  // map->odom = map->base_link * base_link->odom
  const tf2::Transform T_map_odom = T_map_base * T_base_odom;

  // Publish map -> odom TF (required by Nav2)
  geometry_msgs::msg::TransformStamped tf_out;
  tf_out.header.stamp    = stamp;
  tf_out.header.frame_id = "map";
  tf_out.child_frame_id  = "odom";
  tf_out.transform       = tf2::toMsg(T_map_odom);
  tf_broadcaster_->sendTransform(tf_out);

  // Publish /odometry/global
  // Pose: GPS x, y + confident yaw.
  // Covariance diagonal: [x, y, z, roll, pitch, yaw] at indices 0,7,14,21,28,35.
  nav_msgs::msg::Odometry odom;
  odom.header.stamp        = stamp;
  odom.header.frame_id     = "map";
  odom.child_frame_id      = "base_link";
  odom.pose.pose.position.x  = *last_x_;
  odom.pose.pose.position.y  = *last_y_;
  odom.pose.pose.position.z  = 0.0;
  odom.pose.pose.orientation = tf2::toMsg(q);
  odom.pose.covariance[0]  = gps_pos_variance_;  // x
  odom.pose.covariance[7]  = gps_pos_variance_;  // y
  odom.pose.covariance[14] = 1e6;                // z (not tracked)
  odom.pose.covariance[21] = 1e6;                // roll (not tracked)
  odom.pose.covariance[28] = 1e6;                // pitch (not tracked)
  odom.pose.covariance[35] = yaw_variance_;      // yaw
  odom_pub_->publish(odom);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsFusionNode>());
  rclcpp::shutdown();
  return 0;
}
