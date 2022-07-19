#pragma once

#include <memory>
#include <sstream>

#define LOG_DEBUG(logger, str) \
  if (logger) {                \
    std::stringstream ss;      \
    ss << str;                 \
    logger->logDebug(ss);      \
  }
#define LOG_INFO(logger, str) \
  if (logger) {               \
    std::stringstream ss;     \
    ss << str;                \
    logger->logInfo(ss);      \
  }
#define LOG_WARN(logger, str) \
  if (logger) {               \
    std::stringstream ss;     \
    ss << str;                \
    logger->logWarning(ss);   \
  }
#define LOG_ERROR(logger, str) \
  if (logger) {                \
    std::stringstream ss;      \
    ss << str;                 \
    logger->logError(ss);      \
  }
#define LOG_FATAL(logger, str) \
  if (logger) {                \
    std::stringstream ss;      \
    ss << str;                 \
    logger->logFatal(ss);      \
  }

class IIssueLogger {
 public:
  virtual void logDebug(const std::stringstream& ss) = 0;
  virtual void logInfo(const std::stringstream& ss) = 0;
  virtual void logWarning(const std::stringstream& ss) = 0;
  virtual void logError(const std::stringstream& ss) = 0;
  virtual void logFatal(const std::stringstream& ss) = 0;
};

using LoggerPtr = std::shared_ptr<IIssueLogger>;
