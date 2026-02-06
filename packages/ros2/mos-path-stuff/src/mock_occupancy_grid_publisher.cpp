#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MockOccupancyGridPublisher : public rclcpp::Node {
public:
  MockOccupancyGridPublisher() : rclcpp::Node("mock_occupancy_grid_publisher") {
    topic_ = this->declare_parameter<std::string>("topic", "/fused_grid");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    width_ = this->declare_parameter<int>("width", 200);
    height_ = this->declare_parameter<int>("height", 200);
    resolution_ = this->declare_parameter<double>("resolution", 0.05);
    origin_x_ = this->declare_parameter<double>("origin_x", -5.0);
    origin_y_ = this->declare_parameter<double>("origin_y", -5.0);
    default_value_ = this->declare_parameter<int>("default_value", 0);
    publish_hz_ = this->declare_parameter<double>("publish_hz", 1.0);

    auto qos = rclcpp::QoS(1).transient_local().reliable();
    pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(topic_, qos);

    msg_.header.frame_id = frame_id_;
    msg_.info.resolution = resolution_;
    msg_.info.width = static_cast<uint32_t>(width_);
    msg_.info.height = static_cast<uint32_t>(height_);
    msg_.info.origin.position.x = origin_x_;
    msg_.info.origin.position.y = origin_y_;
    msg_.info.origin.orientation.w = 1.0;
    msg_.data.assign(static_cast<size_t>(width_ * height_), static_cast<int8_t>(default_value_));

    auto period = std::chrono::duration<double>(1.0 / (publish_hz_ > 0.0 ? publish_hz_ : 1.0));
    timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this]() {
      msg_.header.stamp = this->now();
      pub_->publish(msg_);
    });
  }

private:
  std::string topic_;
  std::string frame_id_;
  int width_;
  int height_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  int default_value_;
  double publish_hz_;

  nav_msgs::msg::OccupancyGrid msg_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockOccupancyGridPublisher>());
  rclcpp::shutdown();
  return 0;
}
