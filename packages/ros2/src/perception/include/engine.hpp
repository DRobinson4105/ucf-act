#pragma once

#include <array>
#include <string>
#include <vector>
#include <memory>
#include <stdexcept>
#include <filesystem>
#include <algorithm>
#include <fstream>

#include <cuda_runtime.h>
#include <NvInfer.h>
#include <opencv2/core.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudawarping.hpp>

#include "util.hpp"

using OutputType = float;

class Engine {
public:
  explicit Engine(act::util::Logger& logger) : m_logger(logger), m_trt_logger(logger) {};

  bool loadNetwork(const std::string &trtModelPath, const std::array<float, 3> &subVals = {0.f, 0.f, 0.f}, const std::array<float, 3> &divVals = {1.f, 1.f, 1.f}, bool normalize = true);

  bool runInference(const std::vector<std::vector<cv::cuda::GpuMat>> &inputs, std::vector<std::vector<std::vector<OutputType>>> &featureVectors);

  cv::cuda::GpuMat resize(const cv::cuda::GpuMat &input, size_t height, size_t width, const cv::Scalar &bgcolor = cv::Scalar(0, 0, 0));

  cv::cuda::GpuMat blobFromGpuMats(const std::vector<cv::cuda::GpuMat> &batchInput, const std::array<float, 3> &subVals, const std::array<float, 3> &divVals, bool normalize, bool swapRB = false);

private:
  void clearGpuBuffers();

  int m_maxBatchSize = 6;

  std::array<float, 3> m_subVals;
  std::array<float, 3> m_divVals;
  bool m_normalize;

  std::vector<void *> m_buffers;
  std::vector<uint32_t> m_outputLengths{};
  std::vector<nvinfer1::Dims3> m_inputDims;
  std::vector<nvinfer1::Dims> m_outputDims;
  std::vector<std::string> m_IOTensorNames;
  int32_t m_inputBatchSize;

  std::unique_ptr<nvinfer1::IRuntime> m_runtime;
  std::unique_ptr<nvinfer1::ICudaEngine> m_engine;
  std::unique_ptr<nvinfer1::IExecutionContext> m_context;

  act::util::Logger& m_logger;
  act::util::TrtLogger m_trt_logger;
};
