#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <nav2_msgs/msg/speed_limit.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>

class CmdVelAdapter : public rclcpp::Node {
public:
  CmdVelAdapter() : rclcpp::Node("cmd_vel_adapter") {
    cmd_in_topic_ = declare_parameter<std::string>("cmd_in_topic", "/cmd_vel_nav");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry/local");
    can_topic_ = declare_parameter<std::string>("can_topic", "/to_can_bus");
    speed_limit_topic_ = declare_parameter<std::string>("speed_limit_topic", "/speed_limit");

    publish_hz_ = declare_parameter<double>("publish_hz", 50.0);

    v_max_ = declare_parameter<double>("v_max", 3.58);
    w_max_ = declare_parameter<double>("w_max", 1.28);

    a_max_ = declare_parameter<double>("a_max", 1.0);
    d_max_ = declare_parameter<double>("d_max", 1.5);
    w_acc_max_ = declare_parameter<double>("w_acc_max", 2.0);
    speed_limit_timeout_s_ = declare_parameter<double>("speed_limit_timeout_s", 0.5);
    min_turning_r_ = declare_parameter<double>("min_turning_r", 2.8);

    cmd_in_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_in_topic_, 10,
      std::bind(&CmdVelAdapter::onCmd, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&CmdVelAdapter::onOdom, this, std::placeholders::_1));

    speed_limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>(
      speed_limit_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&CmdVelAdapter::onSpeedLimit, this, std::placeholders::_1));

    can_pub_ = create_publisher<can_msgs::msg::Frame>(can_topic_, 10);

    last_time_ = now();
    last_speed_limit_time_ = now();

    const double hz = std::max(1e-3, publish_hz_);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / hz));
    timer_ = create_wall_timer(period, std::bind(&CmdVelAdapter::onTimer, this));
  }

private:
  static double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
  }

  uint16_t normalize(double value, double inMin, double inMax, uint16_t outMin, uint16_t outMax) {
    double clamped = std::clamp(value, inMin, inMax);
    double normalized = (clamped - inMin) / (inMax - inMin);
    return static_cast<uint16_t>(outMin + normalized * (outMax - outMin) + 0.5);
  }

  uint8_t counter_ = 0;
  double decelerateL_ = 1.0;
  double v_acc_min_ = 0.0, v_acc_max_ = 1.0;
  double v_decc_min_ = 0.0, v_decc_max_ = 1.5;
  double w_vel_min_ = -1.28, w_vel_max_ = 1.28;
  uint8_t throttle_min_ = 0, throttle_max_ = 7;
  uint16_t steering_min_ = 0, steering_max_ = 720;
  uint8_t braking_min_ = 0, braking_max_ = 3;

  std::vector<uint8_t> map_values(double v_acc, double w_vel) {
    double accel = v_acc - decelerateL_;
    uint8_t throttle_val = static_cast<uint8_t>(
      normalize(std::max(accel, 0.0), v_acc_min_, v_acc_max_, throttle_min_, throttle_max_));
    uint16_t steering_raw = normalize(w_vel, w_vel_min_, w_vel_max_, steering_min_, steering_max_);

    uint8_t steering_hi = static_cast<uint8_t>((steering_raw >> 8) & 0xFF);
    uint8_t steering_lo = static_cast<uint8_t>(steering_raw & 0xFF);
    uint8_t braking_val = static_cast<uint8_t>(
      normalize(std::max(-accel, 0.0), v_decc_min_, v_decc_max_, braking_min_, braking_max_));
    
    RCLCPP_INFO(get_logger(), "%.3f %.3f %d %d %d", v_acc, w_out, throttle_val, steering_raw, braking_val);

    return {throttle_val, steering_hi, steering_lo, braking_val};
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

  void onSpeedLimit(const nav2_msgs::msg::SpeedLimit::SharedPtr msg) {
    if (!msg) return;
    last_speed_limit_time_ = now();

    if (!std::isfinite(msg->speed_limit) || msg->speed_limit <= 0.0) {
      have_active_speed_cap_ = false;
      active_speed_cap_mps_ = v_max_;
      return;
    }

    double cap = msg->percentage
      ? (v_max_ * msg->speed_limit / 100.0)
      : msg->speed_limit;
    if (!std::isfinite(cap) || cap <= 0.0) {
      have_active_speed_cap_ = false;
      active_speed_cap_mps_ = v_max_;
      return;
    }

    active_speed_cap_mps_ = clamp(cap, 0.0, v_max_);
    have_active_speed_cap_ = true;
  }

  void onTimer() {
    counter_ = counter_ == 255 ? 0 : (counter_ + 1);
    const auto t = now();
    double dt = (t - last_time_).seconds();
    if (dt < 0.0) dt = 0.0;
    if (dt > 0.2) dt = 0.2;
    last_time_ = t;

    if (!have_cmd_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for /cmd_vel_nav before publishing CAN frame");
      return;
    }

    if (have_active_speed_cap_ && (t - last_speed_limit_time_).seconds() > speed_limit_timeout_s_) {
      have_active_speed_cap_ = false;
      active_speed_cap_mps_ = v_max_;
    }

    const double v_raw = cmd_in_.linear.x;
    const double w_raw = cmd_in_.angular.z;

    const double active_speed_cap = have_active_speed_cap_ ? active_speed_cap_mps_ : v_max_;
    const double v_cmd = clamp(v_raw, 0.0, std::min(v_max_, active_speed_cap));
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

    double w_out = w_filt_;
    if (std::abs(v_filt_) <= 1e-3) {
      w_out = 0.0;
    } else {
      const double curv_limit = std::min(w_max_, std::abs(v_filt_) / std::max(1e-6, min_turning_r_));
      w_out = clamp(w_filt_, -curv_limit, curv_limit);
    }

    double v_curr = have_odom_ ? odom_.twist.twist.linear.x : 0.0;
    double v_acc = v_filt_ - v_curr;
    auto mapped = map_values(v_acc, w_out);
 

    can_msgs::msg::Frame can_frame;
    can_frame.header.stamp = t;
    can_frame.header.frame_id = "can";
    can_frame.id = 0x111;
    can_frame.is_extended = false;
    can_frame.is_error = false;
    can_frame.dlc = 5;
    can_frame.data[0] = counter_;
    can_frame.data[1] = mapped[0];  // throttle
    can_frame.data[2] = mapped[1];  // steering high byte
    can_frame.data[3] = mapped[2];  // steering low byte
    can_frame.data[4] = mapped[3];  // braking

    can_pub_->publish(can_frame);
  }

  std::string cmd_in_topic_;
  std::string odom_topic_;
  std::string can_topic_;
  std::string speed_limit_topic_;

  double publish_hz_{50.0};

  double v_max_{3.58};
  double w_max_{1.28};

  double a_max_{1.0};
  double d_max_{1.5};
  double w_acc_max_{2.0};
  double speed_limit_timeout_s_{0.5};
  double min_turning_r_{2.8};

  geometry_msgs::msg::Twist cmd_in_;
  nav_msgs::msg::Odometry odom_;
  bool have_cmd_{false};
  bool have_odom_{false};
  bool have_active_speed_cap_{false};

  bool initialized_{false};
  double v_filt_{0.0};
  double w_filt_{0.0};
  double active_speed_cap_mps_{3.58};

  rclcpp::Time last_time_;
  rclcpp::Time last_speed_limit_time_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_in_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_sub_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelAdapter>());
  rclcpp::shutdown();
  return 0;
}
