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
  FakeOdomIntegrator()
      : rclcpp::Node("fake_odom_integrator"),
        tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)) {

    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odometry/local");
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_nav");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    publish_hz_ = this->declare_parameter<double>("publish_hz", 50.0);

    speed_limit_mps_ = this->declare_parameter<double>("speed_limit_mps", 3.57632);
    accel_limit_mps2_ = this->declare_parameter<double>("accel_limit_mps2", 1.5);
    yaw_rate_limit_rps_ = this->declare_parameter<double>("yaw_rate_limit_rps", 2.0);
    min_turning_r_ = this->declare_parameter<double>("min_turning_r", 1.8);
    allow_reverse_ = this->declare_parameter<bool>("allow_reverse", false);
    max_dt_s_ = this->declare_parameter<double>("max_dt_s", 0.05);

    x_ = this->declare_parameter<double>("initial_x", 0.0);
    y_ = this->declare_parameter<double>("initial_y", 0.0);
    yaw_ = this->declare_parameter<double>("initial_yaw", 0.0);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 10, std::bind(&FakeOdomIntegrator::onCmd, this, std::placeholders::_1));

    last_cmd_.linear.x = 0.0;
    last_cmd_.linear.y = 0.0;
    last_cmd_.linear.z = 0.0;
    last_cmd_.angular.x = 0.0;
    last_cmd_.angular.y = 0.0;
    last_cmd_.angular.z = 0.0;

    v_state_ = 0.0;

    last_time_ = this->now();

    const double hz = std::max(1e-3, publish_hz_);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / hz));
    timer_ = this->create_wall_timer(period, std::bind(&FakeOdomIntegrator::onTimer, this));
  }

private:
  static double clampDouble(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
  }

  static double wrap(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  void onCmd(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!msg) return;
    last_cmd_ = *msg;
  }

  void onTimer() {
    const auto t = this->now();

    if (last_time_.nanoseconds() == 0) {
      last_time_ = t;
      return;
    }

    double dt = (t - last_time_).seconds();
    if (!std::isfinite(dt) || dt <= 0.0) return;
    dt = std::min(dt, std::max(1e-3, max_dt_s_));
    last_time_ = t;

    double v_cmd = last_cmd_.linear.x;
    double wz_cmd = last_cmd_.angular.z;

    if (!allow_reverse_) v_cmd = std::max(0.0, v_cmd);

    v_cmd = clampDouble(v_cmd, -speed_limit_mps_, speed_limit_mps_);

    const double dv_max = accel_limit_mps2_ * dt;
    v_state_ = clampDouble(v_cmd, v_state_ - dv_max, v_state_ + dv_max);

    const double v_abs = std::abs(v_state_);
    double wz_max_from_radius = 0.0;
    if (min_turning_r_ > 1e-6 && v_abs > 1e-3) {
      wz_max_from_radius = v_abs / min_turning_r_;
    }

    double wz_cap = yaw_rate_limit_rps_;
    if (wz_max_from_radius > 0.0) wz_cap = std::min(wz_cap, wz_max_from_radius);

    if (v_abs <= 1e-3) {
      wz_cmd = 0.0;
    } else {
      wz_cmd = clampDouble(wz_cmd, -wz_cap, wz_cap);
    }

    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);

    x_ += v_state_ * c * dt;
    y_ += v_state_ * s * dt;
    yaw_ = wrap(yaw_ + wz_cmd * dt);

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

    odom.twist.twist.linear.x = v_state_;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = wz_cmd;

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

  double speed_limit_mps_{3.57632};
  double accel_limit_mps2_{1.5};
  double yaw_rate_limit_rps_{2.0};
  double min_turning_r_{1.8};
  bool allow_reverse_{false};
  double max_dt_s_{0.05};

  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  geometry_msgs::msg::Twist last_cmd_;
  double v_state_{0.0};

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
