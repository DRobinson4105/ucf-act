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

#include "util.hpp"
#include "engine.hpp"
#include "perception/msg/array.hpp"
#include "perception/msg/array2_d.hpp"
#include "perception/msg/array3_d.hpp"

class SegmentationNode : public rclcpp::Node {
public:
  explicit SegmentationNode();

private:
  void cb(const sensor_msgs::msg::Image::SharedPtr msg);

  perception::msg::Array3D convertToArray(std::vector<OutputType> segMap);

  sensor_msgs::msg::Image convertToImage(std::vector<OutputType> segMap);

  act::util::Logger m_act_logger;
  std::unique_ptr<Engine> engine;

  std::string imageEncoding;
  int width, height, numPixels;

  //rclcpp::Publisher<perception::msg::Array3D>::SharedPtr pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
};
