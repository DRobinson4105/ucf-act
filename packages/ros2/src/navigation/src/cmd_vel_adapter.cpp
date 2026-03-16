#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <navigation/msg/drive_command.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class CmdVelAdapter : public rclcpp::Node {
public:
  CmdVelAdapter() : rclcpp::Node("cmd_vel_adapter") {
    cmd_in_topic_ = declare_parameter<std::string>("cmd_in_topic", "/cmd_vel_nav");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry/local");
    out_topic_ = declare_parameter<std::string>("out_topic", "/act/drive_cmd");

    publish_hz_ = declare_parameter<double>("publish_hz", 50.0);

    v_max_ = declare_parameter<double>("v_max", 3.58);
    w_max_ = declare_parameter<double>("w_max", 1.28);

    a_max_ = declare_parameter<double>("a_max", 1.0);
    d_max_ = declare_parameter<double>("d_max", 1.5);
    w_acc_max_ = declare_parameter<double>("w_acc_max", 2.0);

    cmd_in_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_in_topic_, 10,
      std::bind(&CmdVelAdapter::onCmd, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&CmdVelAdapter::onOdom, this, std::placeholders::_1));

    out_pub_ = create_publisher<navigation::msg::DriveCommand>(out_topic_, 10);

    last_time_ = now();

    const double hz = std::max(1e-3, publish_hz_);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / hz));
    timer_ = create_wall_timer(period, std::bind(&CmdVelAdapter::onTimer, this));
  }

private:
  static double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
  }

  void onCmd(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!msg) return;
    cmd_in_ = *msg;
    have_cmd_ = true;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!msg) return;
    odom_ = *msg;
    have_odom_ = true;
  }

  void onTimer() {
    const auto t = now();
    double dt = (t - last_time_).seconds();
    if (dt < 0.0) dt = 0.0;
    if (dt > 0.2) dt = 0.2;
    last_time_ = t;

    if (!have_cmd_) {
	RCLCPP_INFO_THROTTLE(
	get_logger(), *get_clock(), 2000,
	"Waiting for /cmd_vel_nav before publishing /act/drive_cmd");
	return;
    }

    const double v_raw = cmd_in_.linear.x;
    const double w_raw = cmd_in_.angular.z;

    const double v_cmd = clamp(v_raw, 0.0, v_max_);
    const double w_cmd = clamp(w_raw, -w_max_, w_max_);

    if (!initialized_) {
      v_filt_ = v_cmd;
      w_filt_ = w_cmd;
      initialized_ = true;
    } else {
      const double dv = v_cmd - v_filt_;
      const double dv_lim = (dv >= 0.0) ? (a_max_ * dt) : (d_max_ * dt);
      v_filt_ += clamp(dv, -dv_lim, dv_lim);

      const double dw = w_cmd - w_filt_;
      const double dw_lim = w_acc_max_ * dt;
      w_filt_ += clamp(dw, -dw_lim, dw_lim);
    }

    navigation::msg::DriveCommand out;

    out.header.stamp = t;
    out.header.frame_id = "base_link";
    
    out.v = static_cast<float>(v_filt_);
    out.w = static_cast<float>(w_filt_);
    
    if (have_odom_) {
      out.v_meas = static_cast<float>(odom_.twist.twist.linear.x);
      out.w_meas = static_cast<float>(odom_.twist.twist.angular.z);
      out.v_err = out.v - out.v_meas;
      out.w_err = out.w - out.w_meas;
    } else {
      out.v_meas = 0.0f;
      out.w_meas = 0.0f;
      out.v_err = 0.0f;
      out.w_err = 0.0f;
    }
    
    out_pub_->publish(out);
  }

  std::string cmd_in_topic_;
  std::string odom_topic_;
  std::string out_topic_;

  double publish_hz_{50.0};

  double v_max_{3.58};
  double w_max_{1.28};

  double a_max_{1.0};
  double d_max_{1.5};
  double w_acc_max_{2.0};

  geometry_msgs::msg::Twist cmd_in_;
  nav_msgs::msg::Odometry odom_;
  bool have_cmd_{false};
  bool have_odom_{false};

  bool initialized_{false};
  double v_filt_{0.0};
  double w_filt_{0.0};

  rclcpp::Time last_time_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_in_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<navigation::msg::DriveCommand>::SharedPtr out_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelAdapter>());
  rclcpp::shutdown();
  return 0;
}
