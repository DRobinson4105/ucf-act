#pragma once

#include <deque>
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <limits>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <rmw/qos_profiles.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

class SegCloudNode : public rclcpp::Node {
  public:
    explicit SegCloudNode();

  private:
    void onLidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void onMaskAndInfo(
      const sensor_msgs::msg::Image::ConstSharedPtr mask,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr info
    );

    std::deque<sensor_msgs::msg::PointCloud2> m_rolling_cloud;
    int m_rolling_size;
    double m_ray_dist_threshold;
    int m_non_drivable_id;
    std::string m_camera_frame;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_lidar_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pub;
    
    message_filters::Subscriber<sensor_msgs::msg::Image> m_mask_sub;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> m_info_sub;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;
    std::shared_ptr<Sync> m_sync;

    tf2_ros::Buffer m_tf_buffer;
    tf2_ros::TransformListener m_tf_listener;
};
