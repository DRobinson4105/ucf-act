#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class FakeOdomIntegrator : public rclcpp::Node {
public:
  FakeOdomIntegrator() : rclcpp::Node("fake_odom_integrator"), tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)) {
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odometry/local");
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    publish_hz_ = this->declare_parameter<double>("publish_hz", 50.0);

    x_ = this->declare_parameter<double>("initial_x", 0.0);
    y_ = this->declare_parameter<double>("initial_y", 0.0);
    yaw_ = this->declare_parameter<double>("initial_yaw", 0.0);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10, std::bind(&FakeOdomIntegrator::onCmd, this, std::placeholders::_1));

    last_time_ = this->now();

    const double hz = std::max(1e-3, publish_hz_);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / hz));
    timer_ = this->create_wall_timer(period, std::bind(&FakeOdomIntegrator::onTimer, this));
  }

private:
  static double wrap(double a) {
    while (a > M_PI)
      a -= 2.0 * M_PI;
    while (a < -M_PI)
      a += 2.0 * M_PI;
    return a;
  }

  void onCmd(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!msg) return;
    last_cmd_ = *msg;
  }

  void onTimer() {
    const auto t = this->now();
    double dt = (t - last_time_).seconds();
    if (dt < 0.0) dt = 0.0;
    if (dt > 0.2) dt = 0.2;
    last_time_ = t;

    const double vx = last_cmd_.linear.x;
    const double vy = last_cmd_.linear.y;
    const double wz = last_cmd_.angular.z;

    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);

    x_ += (vx * c - vy * s) * dt;
    y_ += (vx * s + vy * c) * dt;
    yaw_ = wrap(yaw_ + wz * dt);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist = last_cmd_;
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = t;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  double publish_hz_{50.0};

  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_time_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeOdomIntegrator>());
  rclcpp::shutdown();
  return 0;
}
