#pragma once

#include <array>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class GpsLocalFusion : public rclcpp::Node {
  public:
    GpsLocalFusion();

  private:
    struct Pose2D {
      double x {0.0};
      double y {0.0};
      double yaw {0.0};
    };

    struct AnchorState {
      Pose2D gps_pose;
      Pose2D local_pose;
      std::array<double, 36> covariance {};
      rclcpp::Time stamp {0, 0, RCL_ROS_TIME};
    };

    std::string local_odom_topic_;
    std::string gps_odom_topic_;
    std::string output_topic_;

    double publish_hz_;
    double local_stale_timeout_s_;
    double gps_stale_timeout_s_;
    double full_gps_covariance_m2_;
    double zero_gps_covariance_m2_;
    double anchor_covariance_m2_;
    double max_anchor_innovation_m_;
    double dead_reckoning_drift_m2_per_s_;
    int reanchor_after_good_gps_samples_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    nav_msgs::msg::Odometry latest_local_;
    nav_msgs::msg::Odometry latest_gps_;
    bool have_latest_local_ {false};
    bool have_latest_gps_ {false};
    bool latest_gps_gate_open_ {false};

    AnchorState anchor_;
    bool have_anchor_ {false};
    int pending_reanchor_samples_ {0};

    void onLocalOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void onGpsOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publishFusion();

    bool isFresh(const nav_msgs::msg::Odometry &msg, double timeout_s) const;
    bool isFinite(const nav_msgs::msg::Odometry &msg) const;
    bool canPredictFromAnchor() const;

    Pose2D poseFromOdometry(const nav_msgs::msg::Odometry &msg) const;
    Pose2D predictPoseFromAnchor() const;

    double computeGpsWeight(double covariance_m2) const;
    double computeInnovationToPrediction(const nav_msgs::msg::Odometry &gps_msg) const;
    double computePositionVariance(const nav_msgs::msg::Odometry &msg) const;
    double sanitizeVariance(double variance) const;

    void acceptAnchor(const nav_msgs::msg::Odometry &gps_msg);
    nav_msgs::msg::Odometry buildOutputMessage(
      const Pose2D &pose,
      double variance_x,
      double variance_y
    ) const;
};
