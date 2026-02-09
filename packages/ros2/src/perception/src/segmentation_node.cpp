#include "segmentation_node.hpp"

SegmentationNode::SegmentationNode() : rclcpp::Node("segmentation_node"), m_act_logger([this](act::util::Severity severity, const std::string& msg) {
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
  declare_parameter<std::string>("trt_model_path", "");
  declare_parameter<std::string>("out_topic", "/seg_mask");
  declare_parameter<std::string>("in_topic", "/image_raw");
  declare_parameter<double>("frequency", 1.0);
  declare_parameter<std::vector<double>>("sub_vals", {0.0, 0.0, 0.0});
  declare_parameter<std::vector<double>>("div_vals", {1.0, 1.0, 1.0});
  declare_parameter<bool>("normalize", true);
  declare_parameter<std::string>("image_encoding", "rgb8");
  declare_parameter<int>("width", 1280);
  declare_parameter<int>("height", 720);

  std::string trtModelPath = get_parameter("trt_model_path").as_string();
  std::vector<double> subVals = get_parameter("sub_vals").as_double_array();
  std::vector<double> divVals = get_parameter("div_vals").as_double_array();
  bool normalize = get_parameter("normalize").as_bool();

  imageEncoding = get_parameter("image_encoding").as_string();
  width = get_parameter("width").as_int();
  height = get_parameter("height").as_int();

  numPixels = width * height;

  engine = std::make_unique<Engine>(m_act_logger);

  if (subVals.size() != 3) {
    std::string msg = "sub_vals must be a list of size 3.";
    RCLCPP_ERROR(this->get_logger(), msg.c_str());
    throw std::runtime_error(msg);
  }

  if (divVals.size() != 3) {
    std::string msg = "div_vals must be a list of size 3.";
    RCLCPP_ERROR(this->get_logger(), msg.c_str());
    throw std::runtime_error(msg);
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
  
  engine->loadNetwork(trtModelPath, subFloats, divFloats, normalize);

  std::string out_topic = get_parameter("out_topic").as_string();
  std::string in_topic = get_parameter("in_topic").as_string();
  double frequency = get_parameter("frequency").as_double();

  //pub = create_publisher<perception::msg::Array3D>(out_topic, frequency);
  pub = create_publisher<sensor_msgs::msg::Image>(out_topic, frequency);
  sub = create_subscription<sensor_msgs::msg::Image>(
    in_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&SegmentationNode::cb, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "Segmentation node started.");
}

perception::msg::Array3D SegmentationNode::convertToArray(std::vector<OutputType> segMap) { 
  perception::msg::Array3D out;
  
  out.mats.reserve(1);

  perception::msg::Array2D mat_msg;
  mat_msg.rows.reserve(height);

  for (int j = 0; j < height; j++) {
    perception::msg::Array row_msg;
    row_msg.data.reserve(width);

    for (int k = 0; k < width; k++) {
      OutputType cl0 = segMap[j * width + k];
      OutputType cl1 = segMap[numPixels + j * width + k];
      row_msg.data.push_back(cl0 > cl1 ? 0 : 1);
    }

    mat_msg.rows.push_back(std::move(row_msg));
  }

  out.mats.push_back(std::move(mat_msg));

  return out;
}

sensor_msgs::msg::Image SegmentationNode::convertToImage(std::vector<OutputType> segMap) {
  sensor_msgs::msg::Image out;
  out.height = height;
  out.width = width;
  out.encoding = "mono8";
  out.is_bigendian = false;
  out.step = width;
  out.data.resize(numPixels);

  const OutputType* cl0 = segMap.data();
  const OutputType* cl1 = segMap.data() + numPixels;

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      const int idx = i * width + j;

      out.data[idx] = (cl0[idx] > cl1[idx]) ? 255 : 0;
    }
  }

  return out;
}

void SegmentationNode::cb(const sensor_msgs::msg::Image::SharedPtr msg) {
  cv_bridge::CvImageConstPtr cvp;
  try {
    cvp = cv_bridge::toCvShare(msg, imageEncoding);
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

  engine->runInference(inputs, featureVectors);

  // perception::msg::Array3D out = convertToArray(featureVectors[0][0]);
  sensor_msgs::msg::Image out = convertToImage(featureVectors[0][0]);
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = msg->header.frame_id;
  
  pub->publish(out);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  auto node = std::make_shared<SegmentationNode>();
  
  exec->add_node(node);
  exec->spin();
  exec->remove_node(node);
  node.reset();

  rclcpp::shutdown();
  return 0;
}
