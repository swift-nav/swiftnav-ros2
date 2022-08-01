#pragma once

#include <memory>
#include <sstream>

#define LOG_FUNC(logger, str, fn) \
  do {                            \
    if (logger) {                 \
      std::stringstream ss;       \
      ss << str;                  \
      logger->fn(ss);             \
    }                             \
  } while (0)
#define LOG_DEBUG(logger, str) LOG_FUNC(logger, str, logDebug)
#define LOG_INFO(logger, str) LOG_FUNC(logger, str, logInfo)
#define LOG_WARN(logger, str) LOG_FUNC(logger, str, logWarning)
#define LOG_ERROR(logger, str) LOG_FUNC(logger, str, logError)
#define LOG_FATAL(logger, str) LOG_FUNC(logger, str, logFatal)

class IIssueLogger {
 public:
  virtual void logDebug(const std::stringstream& ss) = 0;
  virtual void logInfo(const std::stringstream& ss) = 0;
  virtual void logWarning(const std::stringstream& ss) = 0;
  virtual void logError(const std::stringstream& ss) = 0;
  virtual void logFatal(const std::stringstream& ss) = 0;
};

using LoggerPtr = std::shared_ptr<IIssueLogger>;
