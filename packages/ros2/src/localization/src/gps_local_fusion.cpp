#include "gps_local_fusion.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace {

constexpr double kLargeVariance = 1e6;

double clamp01(double value) {
  return std::max(0.0, std::min(1.0, value));
}

double planarDistance(double ax, double ay, double bx, double by) {
  const double dx = ax - bx;
  const double dy = ay - by;
  return std::hypot(dx, dy);
}

}  

GpsLocalFusion::GpsLocalFusion()
: rclcpp::Node("gps_local_fusion_node") {
  declare_parameter<std::string>("local_odom_topic", "/odometry/local");
  declare_parameter<std::string>("gps_odom_topic", "/odometry/gps");
  declare_parameter<std::string>("output_topic", "/odometry/gps_fused");
  declare_parameter<double>("publish_hz", 20.0);
  declare_parameter<double>("local_stale_timeout_s", 0.5);
  declare_parameter<double>("gps_stale_timeout_s", 1.0);
  declare_parameter<double>("full_gps_covariance_m2", 0.5);
  declare_parameter<double>("zero_gps_covariance_m2", 9.0);
  declare_parameter<double>("anchor_covariance_m2", 0.5);
  declare_parameter<double>("max_anchor_innovation_m", 3.0);
  declare_parameter<double>("dead_reckoning_drift_m2_per_s", 0.05);
  declare_parameter<int>("reanchor_after_good_gps_samples", 3);

  local_odom_topic_ = get_parameter("local_odom_topic").as_string();
  gps_odom_topic_ = get_parameter("gps_odom_topic").as_string();
  output_topic_ = get_parameter("output_topic").as_string();
  publish_hz_ = std::max(1.0, get_parameter("publish_hz").as_double());
  local_stale_timeout_s_ = std::max(0.05, get_parameter("local_stale_timeout_s").as_double());
  gps_stale_timeout_s_ = std::max(0.05, get_parameter("gps_stale_timeout_s").as_double());
  full_gps_covariance_m2_ = sanitizeVariance(get_parameter("full_gps_covariance_m2").as_double());
  zero_gps_covariance_m2_ = sanitizeVariance(get_parameter("zero_gps_covariance_m2").as_double());
  anchor_covariance_m2_ = sanitizeVariance(get_parameter("anchor_covariance_m2").as_double());
  max_anchor_innovation_m_ = std::max(0.0, get_parameter("max_anchor_innovation_m").as_double());
  dead_reckoning_drift_m2_per_s_ = sanitizeVariance(
    get_parameter("dead_reckoning_drift_m2_per_s").as_double()
  );
  reanchor_after_good_gps_samples_ = std::max<int>(
    1,
    static_cast<int>(get_parameter("reanchor_after_good_gps_samples").as_int())
  );

  if (zero_gps_covariance_m2_ < full_gps_covariance_m2_) {
    zero_gps_covariance_m2_ = full_gps_covariance_m2_;
  }

  fused_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);

  local_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    local_odom_topic_,
    10,
    std::bind(&GpsLocalFusion::onLocalOdometry, this, std::placeholders::_1)
  );

  gps_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    gps_odom_topic_,
    10,
    std::bind(&GpsLocalFusion::onGpsOdometry, this, std::placeholders::_1)
  );

  const auto publish_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_hz_)
  );
  publish_timer_ = create_wall_timer(
    publish_period,
    std::bind(&GpsLocalFusion::publishFusion, this)
  );

  RCLCPP_INFO(
    get_logger(),
    "Fusing %s and %s into %s (anchor_cov=%.3f m^2, blend %.3f -> %.3f m^2)",
    local_odom_topic_.c_str(),
    gps_odom_topic_.c_str(),
    output_topic_.c_str(),
    anchor_covariance_m2_,
    full_gps_covariance_m2_,
    zero_gps_covariance_m2_
  );
}

void GpsLocalFusion::onLocalOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (!isFinite(*msg)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ignoring local odometry with non-finite pose"
    );
    return;
  }

  latest_local_ = *msg;
  have_latest_local_ = true;
}

void GpsLocalFusion::onGpsOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (!isFinite(*msg)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ignoring GPS odometry with non-finite pose"
    );
    return;
  }

  latest_gps_ = *msg;
  have_latest_gps_ = true;
  latest_gps_gate_open_ = true;

  if (!have_latest_local_ || !isFresh(latest_local_, local_stale_timeout_s_)) {
    pending_reanchor_samples_ = 0;
    return;
  }

  const double gps_variance = computePositionVariance(*msg);
  const double innovation = computeInnovationToPrediction(*msg);
  if (have_anchor_ && innovation > max_anchor_innovation_m_) {
    latest_gps_gate_open_ = false;

    if (gps_variance <= anchor_covariance_m2_) {
      pending_reanchor_samples_++;

      if (pending_reanchor_samples_ >= reanchor_after_good_gps_samples_) {
        RCLCPP_WARN(
          get_logger(),
          "Accepting large GPS correction of %.2f m after %d consistent low-covariance samples",
          innovation,
          pending_reanchor_samples_
        );
        pending_reanchor_samples_ = 0;
        latest_gps_gate_open_ = true;
        acceptAnchor(*msg);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Rejecting GPS correction of %.2f m until it stays consistent for %d low-covariance samples",
          innovation,
          reanchor_after_good_gps_samples_
        );
      }
    } else {
      pending_reanchor_samples_ = 0;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting GPS correction of %.2f m because covariance %.3f m^2 is above anchor threshold %.3f m^2",
        innovation,
        gps_variance,
        anchor_covariance_m2_
      );
    }
    return;
  }

  pending_reanchor_samples_ = 0;

  if (gps_variance <= anchor_covariance_m2_) {
    pending_reanchor_samples_ = 0;
    acceptAnchor(*msg);
  }
}

void GpsLocalFusion::publishFusion() {
  const bool local_fresh = have_latest_local_ && isFresh(latest_local_, local_stale_timeout_s_);
  const bool gps_fresh = have_latest_gps_ && isFresh(latest_gps_, gps_stale_timeout_s_);

  if (!local_fresh && !gps_fresh) {
    return;
  }

  Pose2D fused_pose;
  double variance_x = kLargeVariance;
  double variance_y = kLargeVariance;

  const bool can_predict = have_anchor_ && local_fresh;
  const Pose2D predicted_pose = can_predict ? predictPoseFromAnchor() :
    (gps_fresh ? poseFromOdometry(latest_gps_) : poseFromOdometry(latest_local_));

  if (can_predict) {
    const rclcpp::Time now = get_clock()->now();
    const double anchor_age_s = std::max(0.0, (now - anchor_.stamp).seconds());
    variance_x = sanitizeVariance(anchor_.covariance[0] + dead_reckoning_drift_m2_per_s_ * anchor_age_s);
    variance_y = sanitizeVariance(anchor_.covariance[7] + dead_reckoning_drift_m2_per_s_ * anchor_age_s);
  }

  fused_pose = predicted_pose;

  if (gps_fresh) {
    const double gps_variance_x = sanitizeVariance(latest_gps_.pose.covariance[0]);
    const double gps_variance_y = sanitizeVariance(latest_gps_.pose.covariance[7]);

    if (!can_predict) {
      fused_pose = poseFromOdometry(latest_gps_);
      variance_x = gps_variance_x;
      variance_y = gps_variance_y;
    } else if (latest_gps_gate_open_) {
      const Pose2D gps_pose = poseFromOdometry(latest_gps_);
      const double weight = computeGpsWeight(computePositionVariance(latest_gps_));

      fused_pose.x = weight * gps_pose.x + (1.0 - weight) * predicted_pose.x;
      fused_pose.y = weight * gps_pose.y + (1.0 - weight) * predicted_pose.y;

      variance_x = weight * gps_variance_x + (1.0 - weight) * variance_x;
      variance_y = weight * gps_variance_y + (1.0 - weight) * variance_y;
    }
  }

  fused_pub_->publish(buildOutputMessage(fused_pose, variance_x, variance_y));
}

bool GpsLocalFusion::isFresh(const nav_msgs::msg::Odometry &msg, double timeout_s) const {
  const auto age = (get_clock()->now() - rclcpp::Time(msg.header.stamp)).seconds();
  return age >= 0.0 && age <= timeout_s;
}

bool GpsLocalFusion::isFinite(const nav_msgs::msg::Odometry &msg) const {
  return std::isfinite(msg.pose.pose.position.x) &&
         std::isfinite(msg.pose.pose.position.y) &&
         std::isfinite(msg.pose.pose.position.z) &&
         std::isfinite(msg.pose.pose.orientation.x) &&
         std::isfinite(msg.pose.pose.orientation.y) &&
         std::isfinite(msg.pose.pose.orientation.z) &&
         std::isfinite(msg.pose.pose.orientation.w);
}

bool GpsLocalFusion::canPredictFromAnchor() const {
  return have_anchor_ && have_latest_local_ && isFresh(latest_local_, local_stale_timeout_s_);
}

GpsLocalFusion::Pose2D GpsLocalFusion::poseFromOdometry(const nav_msgs::msg::Odometry &msg) const {
  Pose2D pose;
  pose.x = msg.pose.pose.position.x;
  pose.y = msg.pose.pose.position.y;

  const tf2::Quaternion q(
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w
  );
  double roll = 0.0;
  double pitch = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, pose.yaw);
  return pose;
}

GpsLocalFusion::Pose2D GpsLocalFusion::predictPoseFromAnchor() const {
  const Pose2D local_pose = poseFromOdometry(latest_local_);

  Pose2D predicted;
  predicted.x = anchor_.gps_pose.x + (local_pose.x - anchor_.local_pose.x);
  predicted.y = anchor_.gps_pose.y + (local_pose.y - anchor_.local_pose.y);
  predicted.yaw = local_pose.yaw;
  return predicted;
}

double GpsLocalFusion::computeGpsWeight(double covariance_m2) const {
  const double safe_covariance = sanitizeVariance(covariance_m2);
  const double range = zero_gps_covariance_m2_ - full_gps_covariance_m2_;

  if (range <= 1e-6) {
    return safe_covariance <= full_gps_covariance_m2_ ? 1.0 : 0.0;
  }

  const double scaled = (zero_gps_covariance_m2_ - safe_covariance) / range;
  return clamp01(scaled);
}

double GpsLocalFusion::computeInnovationToPrediction(const nav_msgs::msg::Odometry &gps_msg) const {
  if (!canPredictFromAnchor()) {
    return 0.0;
  }

  const Pose2D gps_pose = poseFromOdometry(gps_msg);
  const Pose2D predicted_pose = predictPoseFromAnchor();
  return planarDistance(gps_pose.x, gps_pose.y, predicted_pose.x, predicted_pose.y);
}

double GpsLocalFusion::computePositionVariance(const nav_msgs::msg::Odometry &msg) const {
  return std::max(
    sanitizeVariance(msg.pose.covariance[0]),
    sanitizeVariance(msg.pose.covariance[7])
  );
}

double GpsLocalFusion::sanitizeVariance(double variance) const {
  if (!std::isfinite(variance) || variance < 0.0) {
    return kLargeVariance;
  }
  return variance;
}

void GpsLocalFusion::acceptAnchor(const nav_msgs::msg::Odometry &gps_msg) {
  anchor_.gps_pose = poseFromOdometry(gps_msg);
  anchor_.local_pose = poseFromOdometry(latest_local_);
  anchor_.stamp = rclcpp::Time(gps_msg.header.stamp);

  std::copy(
    gps_msg.pose.covariance.begin(),
    gps_msg.pose.covariance.end(),
    anchor_.covariance.begin()
  );

  have_anchor_ = true;
  latest_gps_gate_open_ = true;
}

nav_msgs::msg::Odometry GpsLocalFusion::buildOutputMessage(
  const Pose2D &pose,
  double variance_x,
  double variance_y
) const {
  nav_msgs::msg::Odometry out;

  const bool local_fresh = have_latest_local_ && isFresh(latest_local_, local_stale_timeout_s_);
  const bool gps_fresh = have_latest_gps_ && isFresh(latest_gps_, gps_stale_timeout_s_);

  if (local_fresh) {
    out.header = latest_local_.header;
    out.child_frame_id = latest_local_.child_frame_id;
    out.pose.pose.orientation = latest_local_.pose.pose.orientation;
    out.twist = latest_local_.twist;
  } else if (gps_fresh) {
    out.header = latest_gps_.header;
    out.child_frame_id = latest_gps_.child_frame_id;
    out.pose.pose.orientation = latest_gps_.pose.pose.orientation;
    out.twist = latest_gps_.twist;
  }

  out.pose.pose.position.x = pose.x;
  out.pose.pose.position.y = pose.y;
  out.pose.pose.position.z = 0.0;

  out.pose.covariance.fill(kLargeVariance);
  out.pose.covariance[0] = sanitizeVariance(variance_x);
  out.pose.covariance[7] = sanitizeVariance(variance_y);

  if (local_fresh) {
    out.pose.covariance[35] = sanitizeVariance(latest_local_.pose.covariance[35]);
  } else if (gps_fresh) {
    out.pose.covariance[35] = sanitizeVariance(latest_gps_.pose.covariance[35]);
  }

  return out;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsLocalFusion>());
  rclcpp::shutdown();
  return 0;
}
