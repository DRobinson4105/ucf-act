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

class SegNode : public rclcpp::Node {
public:
  explicit SegNode();

private:
  void cb(const sensor_msgs::msg::Image::SharedPtr msg);

  sensor_msgs::msg::Image convertToImage(std::vector<OutputType> segMap);

  act::util::Logger m_act_logger;
  std::unique_ptr<Engine> m_engine;

  std::string m_imageEncoding;
  int m_width, m_height, m_numPixels;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_sub;
};
