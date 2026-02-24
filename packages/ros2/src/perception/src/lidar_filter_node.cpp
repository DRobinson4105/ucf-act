#include "lidar_filter_node.hpp"

LidarFilter::LidarFilter() : Node("lidar_filter_node") {
  x_min = this->declare_parameter<double>("x_min", -1);
  x_max = this->declare_parameter<double>("x_max", 1);
  y_min = this->declare_parameter<double>("y_min", -1);
  y_max = this->declare_parameter<double>("y_max", 1);

  std::string inTopic = this->declare_parameter<std::string>("in_topic", "/livox/lidar");
  std::string outTopic = this->declare_parameter<std::string>("out_topic", "/lidar_filtered");
  double frequency = this->declare_parameter<double>("frequency", 10);
  bool include = this->declare_parameter<bool>("include", false);

  sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    inTopic,
    rclcpp::SensorDataQoS(),
    std::bind(include ? (&LidarFilter::includeCB) : (&LidarFilter::excludeCB), this, std::placeholders::_1)
  );
  pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(outTopic, frequency);
}

void LidarFilter::includeCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  pcl::PassThrough<pcl::PointXYZ> pass;
  
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(x_min, x_max);
  pass.filter(*cloud);

  pass.setInputCloud(cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(y_min, y_max);
  pass.filter(*cloud);

  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg(*cloud, out_msg);
  out_msg.header = msg->header;
  pub->publish(out_msg);
}

void LidarFilter::excludeCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  pcl::ConditionOr<pcl::PointXYZ>::Ptr condition(new pcl::ConditionOr<pcl::PointXYZ>);
  condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
    new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, x_min)
  ));
  condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
    new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, x_max)
  ));
  condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
    new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, y_min)
  ));
  condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
    new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, y_max)
  ));

  pcl::ConditionalRemoval<pcl::PointXYZ> filter;
  filter.setInputCloud(cloud);
  filter.setCondition(condition);
  filter.filter(*cloud);

  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg(*cloud, out_msg);
  out_msg.header = msg->header;
  pub->publish(out_msg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFilter>());
  rclcpp::shutdown();
}
