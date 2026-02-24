#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>

class LidarFilter : public rclcpp::Node {
public:
  explicit LidarFilter();

private:
  void includeCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void excludeCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  double x_min, x_max, y_min, y_max;
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
};
