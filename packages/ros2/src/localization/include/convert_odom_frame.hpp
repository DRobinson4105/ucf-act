#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class ConvertOdomFrame : public rclcpp::Node {
  public:
    ConvertOdomFrame();

  private:
    std::string in_topic, out_topic;
    std::string in_parent_frame, out_parent_frame, in_child_frame, out_child_frame;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub;
    rclcpp::TimerBase::SharedPtr startup_timer;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    tf2::Transform T_parent;
    tf2::Transform T_child;

    bool have_static_tfs;

    void startupTfCheck();
    bool loadStaticTfs();

    void cb(const nav_msgs::msg::Odometry::SharedPtr msg);

    static tf2::Transform odomMsgPoseToTf(const nav_msgs::msg::Odometry &msg);
};
