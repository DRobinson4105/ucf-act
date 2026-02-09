#include "engine.hpp"

bool Engine::loadNetwork(const std::string &trtModelPath, const std::array<float, 3> &subVals, const std::array<float, 3> &divVals, bool normalize) {
  m_subVals = subVals;
  m_divVals = divVals;
  m_normalize = normalize;

  if (!std::filesystem::exists(trtModelPath)) {
    std::string msg = "Error, unable to read TensorRT model at path: " + trtModelPath;
    m_logger.fatal(msg);
    throw std::runtime_error(msg);
  } else {
    std::string msg = "Loading TensorRT engine file at path: " + trtModelPath;
    m_logger.info(msg);
  }

  std::ifstream file(trtModelPath, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    std::string msg = "Error, unable to open engine file.";
    m_logger.fatal(msg);
    throw std::runtime_error(msg);
  }

  std::streamsize size = file.tellg();
  if (size <= 0) {
    std::string msg = "Error, engine file size invalid (" + std::to_string(size) + ")";
    m_logger.fatal(msg);
    throw std::runtime_error(msg);
  }

  file.seekg(0, std::ios::beg);

  std::vector<char> buffer(size);

  if (!file.read(buffer.data(), size)) {
    std::string msg = "Error, unable to read engine file.";
    m_logger.fatal(msg);
    throw std::runtime_error(msg);
  }

  m_runtime = std::unique_ptr<nvinfer1::IRuntime>{nvinfer1::createInferRuntime(m_trt_logger)};
  if (!m_runtime) {
    std::string msg = "InferenceRuntime failed to load.";
    m_logger.fatal(msg);
    throw std::runtime_error(msg);
  }

  auto ret = cudaSetDevice(0);
  if (ret != 0) {
    std::string msg = "CUDA-capable GPU is unavailable.";
    m_logger.fatal(msg);
    throw std::runtime_error(msg);
  }

  m_engine = std::unique_ptr<nvinfer1::ICudaEngine>(m_runtime->deserializeCudaEngine(buffer.data(), buffer.size()));
  if (!m_engine) {
    std::string msg = "CudaEngine failed to load.";
    m_logger.fatal(msg);
    throw std::runtime_error(msg);
  }

  m_context = std::unique_ptr<nvinfer1::IExecutionContext>(m_engine->createExecutionContext());
  if (!m_context) {
    std::string msg = "ExecutionContext failed to load.";
    m_logger.fatal(msg);
    throw std::runtime_error(msg);
  }

  clearGpuBuffers();
  m_buffers.resize(m_engine->getNbIOTensors());

  m_outputLengths.clear();
  m_inputDims.clear();
  m_outputDims.clear();
  m_IOTensorNames.clear();
  
  cudaStream_t stream;
  act::util::checkCudaErrorCode(m_logger, cudaStreamCreate(&stream));

  m_outputLengths.clear();

  for (int i = 0; i < m_engine->getNbIOTensors(); i++) {
    const auto tensorName = m_engine->getIOTensorName(i);
    m_IOTensorNames.emplace_back(tensorName);
    const auto tensorType = m_engine->getTensorIOMode(tensorName);
    const auto tensorShape = m_engine->getTensorShape(tensorName);

    if (tensorType == nvinfer1::TensorIOMode::kINPUT) {
      if (m_engine->getTensorDataType(tensorName) != nvinfer1::DataType::kFLOAT) {
        std::string msg = "Input must be of type float.";
        m_logger.fatal(msg);
        throw std::runtime_error(msg);
      }

      m_inputDims.emplace_back(tensorShape.d[1], tensorShape.d[2], tensorShape.d[3]);
      m_inputBatchSize = tensorShape.d[0];
    } else if (tensorType == nvinfer1::TensorIOMode::kOUTPUT) {
      uint32_t outputLength = 1;
      m_outputDims.push_back(tensorShape);

      for (int j = 1; j < tensorShape.nbDims; j++) {
        outputLength *= tensorShape.d[j];
      }

      m_outputLengths.push_back(outputLength);
      act::util::checkCudaErrorCode(m_logger, cudaMallocAsync(&m_buffers[i], outputLength * m_maxBatchSize * sizeof(OutputType), stream));
    } else {
      std::string msg = "Error, IO Tensor is neither an input or output";
      m_logger.fatal(msg);
      throw std::runtime_error(msg);
    }
  }

  act::util::checkCudaErrorCode(m_logger, cudaStreamSynchronize(stream));
  act::util::checkCudaErrorCode(m_logger, cudaStreamDestroy(stream));

  return true;
}

bool Engine::runInference(const std::vector<std::vector<cv::cuda::GpuMat>> &inputs, std::vector<std::vector<std::vector<OutputType>>> &featureVectors) {
  if (inputs.empty() || inputs[0].empty()) {
    std::string msg = "Provided input vector is empty.";
    m_logger.error(msg);
    return false;
  }

  const auto numInputs = m_inputDims.size();
  if (inputs.size() != numInputs) {
    std::string msg = "Incorrect number of inputs provided.";
    m_logger.error(msg);
    return false;
  }

  if (inputs[0].size() > static_cast<size_t>(m_maxBatchSize)) {
    std::string msg = "The batch size is larger than the model expects: model max batch size is " +
      std::to_string(m_maxBatchSize) + " and provided batch size is " +
      std::to_string(inputs[0].size()) + ".";
    m_logger.error(msg);
    return false;
  }

  if (m_inputBatchSize != -1 && inputs[0].size() != static_cast<size_t>(m_inputBatchSize)) {
    std::string msg = "The batch size is different than the model expects: model batch size is " +
      std::to_string(m_inputBatchSize) + " and provided batch size is " +
      std::to_string(inputs[0].size()) + ".";
    m_logger.error(msg);
    return false;
  }

  const int32_t batchSize = static_cast<int32_t>(inputs[0].size());

  for (size_t i = 1; i < inputs.size(); i++) {
    if (inputs[i].size() != static_cast<size_t>(batchSize)) {
      std::string msg = "The batch size is different for each input.";
      m_logger.error(msg);
      return false;
    }
  }

  cudaStream_t inferenceCudaStream;
  act::util::checkCudaErrorCode(m_logger, cudaStreamCreate(&inferenceCudaStream));
  std::vector<cv::cuda::GpuMat> preprocessedInputs;

  for (size_t i = 0; i < numInputs; i++) {
    const auto &batchInput = inputs[i];
    const auto &dims = m_inputDims[i];

    auto &input = batchInput[0];

    if (input.channels() != dims.d[0] || input.rows != dims.d[1] || input.cols != dims.d[2]) {
      std::string msg = "Input does not have the correct size: expected (" +
        std::to_string(dims.d[0]) + ", " + std::to_string(dims.d[1]) + ", " +
        std::to_string(dims.d[2]) + ") and received (" + std::to_string(input.channels()) + ", " +
        std::to_string(input.rows) + ", " + std::to_string(input.cols) + ").";
      m_logger.error(msg);
      return false;
    }

    nvinfer1::Dims4 inputDims = {batchSize, dims.d[0], dims.d[1], dims.d[2]};
    m_context->setInputShape(m_IOTensorNames[i].c_str(), inputDims);
    
    auto mfloat = blobFromGpuMats(batchInput, m_subVals, m_divVals, m_normalize);
    preprocessedInputs.push_back(mfloat);
    m_buffers[i] = mfloat.ptr<void>();
  }

  if (!m_context->allInputDimensionsSpecified()) {
    std::string msg = "Not all required dimensions are specified.";
    m_logger.error(msg);
    return false;
  }

  for (size_t i = 0; i < m_buffers.size(); i++) {
    bool status = m_context->setTensorAddress(m_IOTensorNames[i].c_str(), m_buffers[i]);
    if (!status) {
      std::string msg = "TensorAddress could not be set in ExecutionContext.";
      m_logger.error(msg);
      return false;
    }
  }

  bool status = m_context->enqueueV3(inferenceCudaStream);
  if (!status) {
    std::string msg = "Inference could not be started on cuda stream.";
    m_logger.error(msg);
    return false;
  }

  featureVectors.clear();
  for (int batch = 0; batch < batchSize; batch++) {
    std::vector<std::vector<OutputType>> batchOutputs{};

    for (int32_t outputBinding = numInputs; outputBinding < m_engine->getNbIOTensors(); outputBinding++) {
      std::vector<OutputType> output;
      auto outputLength = m_outputLengths[outputBinding - numInputs];
      output.resize(outputLength);
      
      act::util::checkCudaErrorCode(m_logger, cudaMemcpyAsync(output.data(), static_cast<char*>(m_buffers[outputBinding]) + (batch * sizeof(OutputType) * outputLength), outputLength * sizeof(OutputType), cudaMemcpyDeviceToHost, inferenceCudaStream));
      batchOutputs.emplace_back(std::move(output));
    }
    featureVectors.emplace_back(std::move(batchOutputs));
  }

  act::util::checkCudaErrorCode(m_logger, cudaStreamSynchronize(inferenceCudaStream));
  act::util::checkCudaErrorCode(m_logger, cudaStreamDestroy(inferenceCudaStream));

  return true;
}

cv::cuda::GpuMat Engine::resize(const cv::cuda::GpuMat &input, size_t height, size_t width, const cv::Scalar &bgcolor) {
  float r = std::min(width / (input.cols * 1.0), height / (input.rows * 1.0));
  int unpad_w = r * input.cols;
  int unpad_h = r * input.rows;

  cv::cuda::GpuMat re(unpad_h, unpad_w, CV_8UC3);
  cv::cuda::resize(input, re, re.size());
  cv::cuda::GpuMat out(height, width, CV_8UC3, bgcolor);
  re.copyTo(out(cv::Rect(0, 0, re.cols, re.rows)));
  
  return out;
}

cv::cuda::GpuMat Engine::blobFromGpuMats(const std::vector<cv::cuda::GpuMat> &batchInput, const std::array<float, 3> &subVals, const std::array<float, 3> &divVals, bool normalize, bool swapRB) {
  if (batchInput.empty()) {
    std::string msg = "Input is empty.";
    m_logger.error(msg);
    return cv::cuda::GpuMat{};
  }

  if (batchInput[0].channels() != 3) {
    std::string msg = "Input channels must have 3 channels.";
    m_logger.error(msg);
    return cv::cuda::GpuMat{};
  }

  cv::cuda::GpuMat gpu_dst(1, batchInput[0].rows * batchInput[0].cols * batchInput.size(), CV_8UC3);

  size_t width = batchInput[0].cols * batchInput[0].rows;

  if (swapRB) {
    for (size_t img = 0; img < batchInput.size(); img++) {
      std::vector<cv::cuda::GpuMat> input_channels{
        cv::cuda::GpuMat(batchInput[0].rows, batchInput[0].cols, CV_8U, &(gpu_dst.ptr()[width * 2 + width * 3 * img])),
        cv::cuda::GpuMat(batchInput[0].rows, batchInput[0].cols, CV_8U, &(gpu_dst.ptr()[width + width * 3 * img])),
        cv::cuda::GpuMat(batchInput[0].rows, batchInput[0].cols, CV_8U, &(gpu_dst.ptr()[width * 3 * img]))
      };
      cv::cuda::split(batchInput[img], input_channels);
    }
  } else {
    for (size_t img = 0; img < batchInput.size(); img++) {
      std::vector<cv::cuda::GpuMat> input_channels{
        cv::cuda::GpuMat(batchInput[0].rows, batchInput[0].cols, CV_8U, &(gpu_dst.ptr()[width * 3 * img])),
        cv::cuda::GpuMat(batchInput[0].rows, batchInput[0].cols, CV_8U, &(gpu_dst.ptr()[width + width * 3 * img])),
        cv::cuda::GpuMat(batchInput[0].rows, batchInput[0].cols, CV_8U, &(gpu_dst.ptr()[width * 2 + width * 3 * img]))
      };
      cv::cuda::split(batchInput[img], input_channels);
    }
  }

  cv::cuda::GpuMat mfloat;

  if (normalize) {
    gpu_dst.convertTo(mfloat, CV_32FC3, 1.f / 255.f);
  } else {
    gpu_dst.convertTo(mfloat, CV_32FC3);
  }

  cv::cuda::subtract(mfloat, cv::Scalar(subVals[0], subVals[1], subVals[2]), mfloat, cv::noArray(), -1);
  cv::cuda::divide(mfloat, cv::Scalar(divVals[0], divVals[1], divVals[2]), mfloat, 1, -1);

  return mfloat;
}

void Engine::clearGpuBuffers() {
  if (!m_buffers.empty()) {
    const auto numInputs = m_inputDims.size();
    for (int32_t outputBinding = numInputs; outputBinding < m_engine->getNbIOTensors(); outputBinding++) {
      act::util::checkCudaErrorCode(m_logger, cudaFree(m_buffers[outputBinding]));
    }
    m_buffers.clear();
  }
}
