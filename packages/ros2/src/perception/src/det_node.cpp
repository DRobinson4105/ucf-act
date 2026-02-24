#include "det_node.hpp"

DetNode::DetNode() : rclcpp::Node("det_node"), act_logger([this](act::util::Severity severity, const std::string& msg) {
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
  std::string outTopic = this->declare_parameter<std::string>("out_topic", "/objects");
  std::string inTopic = this->declare_parameter<std::string>("in_topic", "/image_raw");
  double frequency = this->declare_parameter<double>("frequency", 1.0);
  std::vector<double> subVals = this->declare_parameter<std::vector<double>>("sub_vals", {0.0, 0.0, 0.0});
  std::vector<double> divVals = this->declare_parameter<std::vector<double>>("div_vals", {1.0, 1.0, 1.0});
  bool normalize = this->declare_parameter<bool>("normalize", true);
  
  this->imageEncoding = this->declare_parameter<std::string>("image_encoding", "rgb8");
  this->confidenceThreshold = this->declare_parameter<double>("confidence_threshold", 0.5);
  this->originalWidth = this->declare_parameter<int>("width", 0);
  this->originalHeight = this->declare_parameter<int>("height", 0);

  this->engine = std::make_unique<Engine>(this->act_logger);
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
  
  if (!this->engine->loadNetwork(trtModelPath, subFloats, divFloats, normalize)) {
    std::string msg = "Engine failed to load.";
    RCLCPP_ERROR(this->get_logger(), msg.c_str());
    return;
  }

  this->numDetections = this->engine->getOutputShape().d[1];
  this->stride = this->engine->getOutputShape().d[2];
  this->inputHeight = this->engine->getInputShape().d[2];
  this->inputWidth = this->engine->getInputShape().d[3];

  this->pub = create_publisher<vision_msgs::msg::Detection2DArray>(outTopic, frequency);
  this->sub = create_subscription<sensor_msgs::msg::Image>(
    inTopic,
    rclcpp::SensorDataQoS(),
    std::bind(&DetNode::cb, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "Detection node started.");
}

void DetNode::cb(const sensor_msgs::msg::Image::SharedPtr msg) {
  cv_bridge::CvImageConstPtr cvp;
  try {
    cvp = cv_bridge::toCvShare(msg, this->imageEncoding);
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
    return;
  }

  const cv::Mat &cpuImg = cvp->image;
  if (cpuImg.empty()) {
    std::string msg = "Failed to copy image to OpenCV.";
    RCLCPP_ERROR(this->get_logger(), msg.c_str());
    return;
  }

  cv::cuda::GpuMat gpuImg;
  gpuImg.upload(cpuImg);

  std::vector<std::vector<cv::cuda::GpuMat>> inputs = {{ gpuImg }};
  std::vector<std::vector<std::vector<OutputType>>> featureVectors;

  if (!this->engine->runInference(inputs, featureVectors)) {
    std::string msg = "Failed to run inference on image.";
    RCLCPP_ERROR(this->get_logger(), msg.c_str());
    return;
  }

  std::vector<float>& output = featureVectors[0][0];

  for (int i = 0; i < 1800; i++) {
    if (output[i] != 0) {
      RCLCPP_INFO(this->get_logger(), std::to_string(output[i]).c_str());
    }
  }

  vision_msgs::msg::Detection2DArray detArray;
  detArray.header = msg->header;

  float scale = std::max(
    this->inputWidth / (float)this->originalWidth,
    this->inputHeight / (float)this->originalHeight
  );

  for (int i = 0; i < this->numDetections; i++) {
    float x1 = output[i * this->stride + 0];
    float y1 = output[i * this->stride + 1];
    float x2 = output[i * this->stride + 2];
    float y2 = output[i * this->stride + 3];
    float conf = output[i * this->stride + 4];
    int cls = static_cast<int>(output[i * this->stride + 5]);

    if (conf < this->confidenceThreshold) {
      continue;
    }

    x1 = std::clamp(x1 / scale, 0.f, (float)originalWidth);
    y1 = std::clamp(y1 / scale, 0.f, (float)originalHeight);
    x2 = std::clamp(x2 / scale, 0.f, (float)originalWidth);
    y2 = std::clamp(y2 / scale, 0.f, (float)originalHeight);

    vision_msgs::msg::Detection2D det;
    det.header = msg->header;
    det.bbox.center.position.x = (x1 + x2) / 2.0f;
    det.bbox.center.position.y = (y1 + y2) / 2.0f;
    det.bbox.size_x = x2 - x1;
    det.bbox.size_y = y2 - y1;

    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.class_id = std::to_string(cls);
    hyp.hypothesis.score = conf;
    det.results.push_back(hyp);

    detArray.detections.push_back(det);
  }

  pub->publish(detArray);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetNode>());
  rclcpp::shutdown();
}
