#pragma once

#include <sstream>
#include <string>
#include <vector>
#include <filesystem>
#include <stdexcept>
#include <functional>

#include <cuda_runtime.h>
#include <NvInfer.h>

namespace act::util {
  enum class Severity { Debug, Info, Warn, Error, Fatal };

  using LogSink = std::function<void(act::util::Severity, const std::string&)>;

  class Logger {
    public:
      explicit Logger(LogSink sink) : m_sink(std::move(sink)) {}
      
      template <typename T>
      void log(act::util::Severity severity, const T& msg);

      template <typename T>
      void debug(const T& msg);
      
      template <typename T>
      void info(const T& msg);

      template <typename T>
      void warn(const T& msg);

      template <typename T>
      void error(const T& msg);

      template <typename T>
      void fatal(const T& msg);

    private:
      LogSink m_sink;
  };

  class TrtLogger : public nvinfer1::ILogger {
    public:
      explicit TrtLogger(act::util::Logger& logger) : m_logger(logger) {}

      void log(nvinfer1::ILogger::Severity severity, const char* msg) noexcept override;

    private:
      act::util::Logger& m_logger;
  };

  void checkCudaErrorCode(act::util::Logger& logger, cudaError_t code);

  std::vector<std::string> listDirectory(const std::string &dirPath);
}
