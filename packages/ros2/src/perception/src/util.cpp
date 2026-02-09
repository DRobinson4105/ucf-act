#include <util.hpp>

namespace act::util {
  template <typename T>
  void Logger::log(act::util::Severity severity, const T& msg) {
    std::ostringstream oss;
    oss << msg;
    m_sink(severity, oss.str());
  }

  template <typename T>
  void Logger::debug(const T& msg) {
    log(act::util::Severity::Debug, msg);
  }
  
  template <typename T>
  void Logger::info(const T& msg) {
    log(act::util::Severity::Info, msg);
  }

  template <typename T>
  void Logger::warn(const T& msg) {
    log(act::util::Severity::Warn, msg);
  }

  template <typename T>
  void Logger::error(const T& msg) {
    log(act::util::Severity::Error, msg);
  }

  template <typename T> 
  void Logger::fatal(const T& msg) {
    log(act::util::Severity::Fatal, msg);
  }

  void TrtLogger::log(nvinfer1::ILogger::Severity severity, const char* msg) noexcept {
    std::string fullMsg = std::string("[TensorRT] ") + (msg ? msg : "");

    switch(severity) {
      case nvinfer1::ILogger::Severity::kINTERNAL_ERROR:
        m_logger.fatal(fullMsg);
        break;
      case nvinfer1::ILogger::Severity::kERROR:
        m_logger.error(fullMsg);
        break;
      case nvinfer1::ILogger::Severity::kWARNING:
        m_logger.warn(fullMsg);
        break;
      case nvinfer1::ILogger::Severity::kINFO:
        m_logger.info(fullMsg);
        break;
      default:
        m_logger.debug(fullMsg);
        break;
    }
  }

   void checkCudaErrorCode(act::util::Logger& logger, cudaError_t code) {
    if (code != cudaSuccess) {
      std::string msg = "CUDA operation failed with code: " + std::to_string(code) + " (" + cudaGetErrorName(code) + "), with message: " + cudaGetErrorString(code);
      logger.error(msg);
      throw std::runtime_error(msg);
    }
  }

  std::vector<std::string> listDirectory(const std::string &dirPath) {
    std::vector<std::string> fileNames;
    for (const auto &entry : std::filesystem::directory_iterator(dirPath)) {
      if (entry.is_regular_file()) {
        fileNames.push_back(entry.path().string());
      }
    }
    return fileNames;
  }
}
