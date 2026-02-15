#include "seg_cloud_node.hpp"

SegCloudNode::SegCloudNode() : rclcpp::Node("seg_cloud_node"), m_tf_buffer(this->get_clock()), m_tf_listener(m_tf_buffer) {
  m_rolling_size = this->declare_parameter<int>("rolling_size", 10);
  m_ray_dist_threshold = this->declare_parameter<double>("ray_dist_threshold", 0.25);
  m_non_drivable_id = this->declare_parameter<int>("non_drivable_class_id", 0);
  m_camera_frame = this->declare_parameter<std::string>("camera_frame", "camera_frame");

  std::string lidar_topic = this->declare_parameter<std::string>("lidar_topic", "/lidar");
  std::string mask_topic = this->declare_parameter<std::string>("mask_topic", "/mask");
  std::string info_topic = this->declare_parameter<std::string>("camera_info_topic", "/camera_info");
  std::string out_topic = this->declare_parameter<std::string>("out_topic", "/mask_pc");
  double frequency = this->declare_parameter<double>("frequency", 10);

  m_lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic, rclcpp::SensorDataQoS(),
    std::bind(&SegCloudNode::onLidar, this, std::placeholders::_1)
  );

  m_mask_sub.subscribe(this, mask_topic, rmw_qos_profile_sensor_data);
  m_info_sub.subscribe(this, info_topic, rmw_qos_profile_sensor_data);

  m_sync.reset(new Sync(SyncPolicy(10), m_mask_sub, m_info_sub));
  m_sync->registerCallback(std::bind(
    &SegCloudNode::onMaskAndInfo, this,
    std::placeholders::_1, std::placeholders::_2
  ));

  m_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(out_topic, frequency);
  
  RCLCPP_INFO(this->get_logger(), "Segmentation cloud node started.");
}

void SegCloudNode::onLidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  m_rolling_cloud.push_back(*msg);
  if ((int) m_rolling_cloud.size() > m_rolling_size) {
    m_rolling_cloud.pop_front();
  }
}

void SegCloudNode::onMaskAndInfo(
  const sensor_msgs::msg::Image::ConstSharedPtr mask,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr info
) {
  if (m_rolling_cloud.empty()) {
    return;
  }

  const int width = (int) info->width;
  const int height = (int)info->height;

  const double fx = info->k[0];
  const double fy = info->k[4];
  const double cx = info->k[2];
  const double cy = info->k[5];

  if (mask->encoding != "mono8") {
    RCLCPP_WARN(this->get_logger(), "Mask encoding not mono8.");
    return;
  }

  if ((int) mask->width != width || (int) mask->height != height) {
    RCLCPP_WARN(this->get_logger(), "Mask size != camera info size.");
    return;
  }

  geometry_msgs::msg::TransformStamped tf_cam_from_lidar;

  try {
    tf_cam_from_lidar = m_tf_buffer.lookupTransform(
      m_camera_frame,
      m_rolling_cloud.back().header.frame_id,
      mask->header.stamp
    );
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return;
  }

  const Eigen::Isometry3d T = tf2::transformToEigen(tf_cam_from_lidar);

  const int N = width * height;
  std::vector<float> best_t(N, std::numeric_limits<float>::infinity());
  std::vector<Eigen::Vector3f> best_p(N, Eigen::Vector3f::Zero());

  auto mask_at = [&](int u, int v) -> uint8_t {
    const size_t idx = (size_t)v * mask->step + (size_t)u;
    return mask->data[idx];
  };

  for (const auto& cloud : m_rolling_cloud) {
    sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      Eigen::Vector3d pl(*it_x, *it_y, *it_z);
      Eigen::Vector3d pc = T * pl;

      const double x = pc.x(), y = pc.y(), z = pc.z();
      if (x <= 0.0) {
        continue;
      }

      const int u = (int) std::lround(-fx * y / x + cx);
      const int v = (int) std::lround(-fy * z / x + cy);

      if (u < 0 || u >= width || v < 0 || v >= height) {
        continue;
      }

      if (mask_at(u, v) != (uint8_t) m_non_drivable_id) {
        continue;
      }

      Eigen::Vector3d r(1.0, -(u - cx) / fx, -(v - cy) / fy);
      Eigen::Vector3d d = r.normalized();
      
      const double t = pc.dot(d);
      if (t <= 0.0) {
        continue;
      }

      const Eigen::Vector3d perp = pc - t * d;
      const double dist = perp.norm();

      if (dist > m_ray_dist_threshold) {
        continue;
      }

      const int pix = v * width + u;
      if ((float) t < best_t[pix]) {
        best_t[pix] = (float)t;
        best_p[pix] = pc.cast<float>();
      }
    }
  }

  sensor_msgs::msg::PointCloud2 out;
  out.header.stamp = mask->header.stamp;
  out.header.frame_id = m_camera_frame;

  size_t count = 0;
  for (int i = 0; i < N; i++) {
    if (std::isfinite(best_t[i])) {
      count++;
    }
  }

  sensor_msgs::PointCloud2Modifier mod(out);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(count);

  sensor_msgs::PointCloud2Iterator<float> out_x(out, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(out, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(out, "z");

  for (int i = 0; i < N; i++) {
    if (!std::isfinite(best_t[i])) {
      continue;
    }

    const auto& p = best_p[i];
    
    *out_x = p.x();
    ++out_x;

    *out_y = p.y();
    ++out_y;

    *out_z = p.z();
    ++out_z;
  }

  m_pub->publish(out);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  auto node = std::make_shared<SegCloudNode>();

  exec->add_node(node);
  exec->spin();
  exec->remove_node(node);
  node.reset();

  rclcpp::shutdown();
  return 0;
}
