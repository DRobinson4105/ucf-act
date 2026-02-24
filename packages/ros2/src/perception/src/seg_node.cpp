#include "seg_node.hpp"

SegNode::SegNode() : rclcpp::Node("seg_node"), m_act_logger([this](act::util::Severity severity, const std::string& msg) {
  switch (severity) {
    case act::util::Severity::Fatal:
      RCLCPP_FATAL(this->get_logger(), msg.c_str());
      break;
    case act::util::Severity::Error:
      RCLCPP_ERROR(this->get_logger(), msg.c_str());
      break;
    case act::util::Severity::Warn:
      RCLCPP_WARN(this->get_logger(), msg.c_str());
      break;
    case act::util::Severity::Info:
      RCLCPP_INFO(this->get_logger(), msg.c_str());
      break;
    case act::util::Severity::Debug:
    default:
      RCLCPP_DEBUG(this->get_logger(), msg.c_str());
      break;
  }
}) {
  std::string trtModelPath = this->declare_parameter<std::string>("trt_model_path", "");
  std::string outTopic = this->declare_parameter<std::string>("out_topic", "/seg_mask");
  std::string inTopic = this->declare_parameter<std::string>("in_topic", "/image_raw");
  double frequency = this->declare_parameter<double>("frequency", 1.0);
  std::vector<double> subVals = this->declare_parameter<std::vector<double>>("sub_vals", {0.0, 0.0, 0.0});
  std::vector<double> divVals = this->declare_parameter<std::vector<double>>("div_vals", {1.0, 1.0, 1.0});
  bool normalize = this->declare_parameter<bool>("normalize", true);
  
  m_imageEncoding = this->declare_parameter<std::string>("image_encoding", "rgb8");
  m_width = this->declare_parameter<int>("width", 1280);
  m_height = this->declare_parameter<int>("height", 720);

  m_numPixels = m_width * m_height;

  m_engine = std::make_unique<Engine>(m_act_logger);

  if (subVals.size() != 3) {
    std::string msg = "sub_vals must be a list of size 3.";
    RCLCPP_ERROR(this->get_logger(), msg.c_str());
    return;
  }

  if (divVals.size() != 3) {
    std::string msg = "div_vals must be a list of size 3.";
    RCLCPP_ERROR(this->get_logger(), msg.c_str());
    return;
  }

  std::array<float, 3> subFloats{
    static_cast<float>(subVals[0]),
    static_cast<float>(subVals[1]),
    static_cast<float>(subVals[2]),
  };
  std::array<float, 3> divFloats{
    static_cast<float>(divVals[0]),
    static_cast<float>(divVals[1]),
    static_cast<float>(divVals[2]),
  };
  
  if (!m_engine->loadNetwork(trtModelPath, subFloats, divFloats, normalize)) {
    std::string msg = "Engine failed to load.";
    RCLCPP_ERROR(this->get_logger(), msg.c_str());
    return;
  }

  m_pub = create_publisher<sensor_msgs::msg::Image>(outTopic, frequency);
  m_sub = create_subscription<sensor_msgs::msg::Image>(
    inTopic,
    rclcpp::SensorDataQoS(),
    std::bind(&SegNode::cb, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "Segmentation node started.");
}

sensor_msgs::msg::Image SegNode::convertToImage(std::vector<OutputType> segMap) {
  sensor_msgs::msg::Image out;
  out.height = m_height;
  out.width = m_width;
  out.encoding = "mono8";
  out.is_bigendian = false;
  out.step = m_width;
  out.data.resize(m_numPixels);

  const OutputType* cl0 = segMap.data();
  const OutputType* cl1 = segMap.data() + m_numPixels;

  for (int i = 0; i < m_height; i++) {
    for (int j = 0; j < m_width; j++) {
      const int idx = i * m_width + j;

      out.data[idx] = (cl0[idx] > cl1[idx]) ? 255 : 0;
    }
  }

  return out;
}

void SegNode::cb(const sensor_msgs::msg::Image::SharedPtr msg) {
  cv_bridge::CvImageConstPtr cvp;
  try {
    cvp = cv_bridge::toCvShare(msg, m_imageEncoding);
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
    return;
  }

  const cv::Mat &cpuImg = cvp->image;
  if (cpuImg.empty()) {
    return;
  }

  cv::cuda::GpuMat gpuImg;
  gpuImg.upload(cpuImg);

  std::vector<std::vector<cv::cuda::GpuMat>> inputs = {{ gpuImg }};
  std::vector<std::vector<std::vector<OutputType>>> featureVectors;

  if (!m_engine->runInference(inputs, featureVectors)) {
    return;
  }

  sensor_msgs::msg::Image out = convertToImage(featureVectors[0][0]);
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = msg->header.frame_id;
  
  m_pub->publish(out);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SegNode>());
  rclcpp::shutdown();
}
