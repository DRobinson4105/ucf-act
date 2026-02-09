#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

struct XY {
  double x;
  double y;
};

class MockLivoxPointcloudPublisher : public rclcpp::Node {
public:
  MockLivoxPointcloudPublisher() : rclcpp::Node("mock_livox_pointcloud_publisher"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    topic_ = this->declare_parameter<std::string>("topic", "/livox/lidar_pc2");
    world_frame_ = this->declare_parameter<std::string>("world_frame", "odom");
    sensor_frame_ = this->declare_parameter<std::string>("sensor_frame", "livox_frame");
    publish_hz_ = this->declare_parameter<double>("publish_hz", 10.0);
    point_z_ = this->declare_parameter<double>("point_z", 1.0);
    points_per_obstacle_ = this->declare_parameter<int>("points_per_obstacle", 24);
    obstacle_radius_ = this->declare_parameter<double>("obstacle_radius", 0.3);
    use_tf_ = this->declare_parameter<bool>("use_tf", true);

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_, 10);
    obs_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/mock_obstacles", 10,
                                                                        std::bind(&MockLivoxPointcloudPublisher::onObstacles, this, std::placeholders::_1));

    const double hz = std::max(1e-3, publish_hz_);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / hz));
    timer_ = this->create_wall_timer(period, std::bind(&MockLivoxPointcloudPublisher::onTimer, this));
  }

private:
  void onObstacles(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    if (!msg) return;
    std::vector<XY> next;
    next.reserve(msg->poses.size());
    for (const auto &p : msg->poses) {
      next.push_back(XY{p.position.x, p.position.y});
    }
    obstacles_.swap(next);
  }

  void onTimer() {
    std::vector<XY> obs = obstacles_;

    tf2::Transform T;
    bool have_tf = false;

    if (use_tf_) {
      try {
        const auto tf_msg = tf_buffer_.lookupTransform(sensor_frame_, world_frame_, tf2::TimePointZero);
        tf2::fromMsg(tf_msg.transform, T);
        have_tf = true;
      } catch (const std::exception &) { have_tf = false; }
    }

    const int per = std::max(1, points_per_obstacle_);
    const size_t total_points = obs.size() * static_cast<size_t>(per);

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = this->now();
    cloud.header.frame_id = sensor_frame_;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(total_points);
    cloud.is_bigendian = false;
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(total_points);

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");

    for (const auto &c : obs) {
      for (int i = 0; i < per; i++) {
        const double a = (2.0 * M_PI) * (static_cast<double>(i) / static_cast<double>(per));
        double wx = c.x + obstacle_radius_ * std::cos(a);
        double wy = c.y + obstacle_radius_ * std::sin(a);
        double wz = point_z_;

        double sx = wx;
        double sy = wy;
        double sz = wz;

        if (use_tf_ && have_tf) {
          const tf2::Vector3 p_world(wx, wy, wz);
          const tf2::Vector3 p_sensor = T * p_world;
          sx = p_sensor.x();
          sy = p_sensor.y();
          sz = p_sensor.z();
        }

        *it_x = static_cast<float>(sx);
        *it_y = static_cast<float>(sy);
        *it_z = static_cast<float>(sz);

        ++it_x;
        ++it_y;
        ++it_z;
      }
    }

    cloud_pub_->publish(cloud);
  }

  std::string topic_;
  std::string world_frame_;
  std::string sensor_frame_;
  double publish_hz_{10.0};
  double point_z_{1.0};
  int points_per_obstacle_{24};
  double obstacle_radius_{0.3};
  bool use_tf_{true};

  std::vector<XY> obstacles_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obs_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockLivoxPointcloudPublisher>());
  rclcpp::shutdown();
  return 0;
}
