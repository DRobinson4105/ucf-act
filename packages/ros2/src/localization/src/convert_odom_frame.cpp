#include "convert_odom_frame.hpp"

ConvertOdomFrame::ConvertOdomFrame()
: rclcpp::Node("convert_odom_frame_node"),
  have_static_tfs(false) {
  declare_parameter<std::string>("in_topic", "/Odometry");
  declare_parameter<std::string>("out_topic", "/odometry/fastlio");
  declare_parameter<std::string>("in_parent_frame", "camera_init");
  declare_parameter<std::string>("out_parent_frame", "odom");
  declare_parameter<std::string>("in_child_frame", "body");
  declare_parameter<std::string>("out_child_frame", "base_link");

 in_topic = get_parameter("in_topic").as_string();
 out_topic = get_parameter("out_topic").as_string();
  in_parent_frame = get_parameter("in_parent_frame").as_string();
  out_parent_frame = get_parameter("out_parent_frame").as_string();
  in_child_frame = get_parameter("in_child_frame").as_string();
  out_child_frame = get_parameter("out_child_frame").as_string();

  tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface()
  );
  tf_buffer->setCreateTimerInterface(timer_interface);

  pub = create_publisher<nav_msgs::msg::Odometry>(out_topic, 10);
  sub = create_subscription<nav_msgs::msg::Odometry>(
    in_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&ConvertOdomFrame::cb, this, std::placeholders::_1)
  );

  startup_timer = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ConvertOdomFrame::startupTfCheck, this)
  );

  RCLCPP_INFO(
    get_logger(),
    "Subscribing to %s (with frames %s->%s), publishing to %s (with frames %s->%s)",
    in_topic.c_str(), in_parent_frame.c_str(), in_child_frame.c_str(),
    out_topic.c_str(), out_parent_frame.c_str(), out_child_frame.c_str()
  );
}

void ConvertOdomFrame::startupTfCheck() {
  if (!tf_listener) {
    tf_listener = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer,
      shared_from_this(),
      true
    );
  }

  if (have_static_tfs) {
    startup_timer->cancel();
    return;
  }

  if (loadStaticTfs()) {
    RCLCPP_INFO(get_logger(), "Static TFs loaded");
  }
}

bool ConvertOdomFrame::loadStaticTfs() {
  const auto t0 = tf2::TimePointZero;
  const tf2::Duration timeout = tf2::durationFromSec(2.0);

  if (!tf_buffer->canTransform(out_parent_frame, in_parent_frame, t0, timeout)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Can't find TF %s -> %s",
      out_parent_frame.c_str(), in_parent_frame.c_str()
    );
    return false;
  }

  if (!tf_buffer->canTransform(in_child_frame, out_child_frame, t0, timeout)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Can't find TF %s -> %s",
      in_child_frame.c_str(), out_child_frame.c_str()
    );
    return false;
  }

  try {
    const auto tf_parent_frame = tf_buffer->lookupTransform(
      out_parent_frame, in_parent_frame, t0
    );
    const auto tf_child_frame = tf_buffer->lookupTransform(
      in_child_frame, out_child_frame, t0
    );

    tf2::fromMsg(tf_parent_frame.transform, T_parent);
    tf2::fromMsg(tf_child_frame.transform, T_child);
    
    have_static_tfs = true;

    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "TF lookup failed");
    return false;
  }
}

tf2::Transform ConvertOdomFrame::odomMsgPoseToTf(const nav_msgs::msg::Odometry &msg) {
  tf2::Transform T;
  tf2::Quaternion q(
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w
  );

  tf2::Vector3 t(
    msg.pose.pose.position.x,
    msg.pose.pose.position.y,
    msg.pose.pose.position.z
  );

  T.setOrigin(t);
  T.setRotation(q);
  return T;
}

void ConvertOdomFrame::cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (!have_static_tfs) {
    return;
  }

  tf2::Transform T_inner = odomMsgPoseToTf(*msg);
  
  tf2::Transform T_odom_base = T_parent * T_inner * T_child;

  nav_msgs::msg::Odometry out;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = out_parent_frame;
  out.child_frame_id = out_child_frame;

  out.pose.pose.position.x = T_odom_base.getOrigin().x();
  out.pose.pose.position.y = T_odom_base.getOrigin().y();
  out.pose.pose.position.z = T_odom_base.getOrigin().z();

  tf2::Quaternion q = T_odom_base.getRotation();
  out.pose.pose.orientation.x = q.x();
  out.pose.pose.orientation.y = q.y();
  out.pose.pose.orientation.z = q.z();
  out.pose.pose.orientation.w = q.w();

  out.pose.covariance = msg->pose.covariance;

  out.twist.twist.linear.x = 0.0;
  out.twist.twist.linear.y = 0.0;
  out.twist.twist.linear.z = 0.0;
  out.twist.twist.angular.x = 0.0;
  out.twist.twist.angular.y = 0.0;
  out.twist.twist.angular.z = 0.0;

  pub->publish(out);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConvertOdomFrame>());
  rclcpp::shutdown();
  return 0;
}
