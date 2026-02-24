#pragma once

#include <string>
#include <vector>
#include <array>
#include <memory>
#include <functional>
#include <utility>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/cudaarithm.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "util.hpp"
#include "engine.hpp"

class DetNode : public rclcpp::Node {
public:
  explicit DetNode();

private:
  void cb(const sensor_msgs::msg::Image::SharedPtr msg);
  
  act::util::Logger act_logger;
  std::unique_ptr<Engine> engine;

  std::string imageEncoding;
  int originalWidth, originalHeight, inputWidth, inputHeight, numDetections, stride;
  double confidenceThreshold;

  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
};
