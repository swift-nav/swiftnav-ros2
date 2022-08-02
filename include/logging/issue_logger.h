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

/**
 * @brief Abstract base class for a logging facility
 */
class IIssueLogger {
 public:
  /**
   * @brief Method used to log Debug information
   *
   * @param ss String Stream containing the data to log
   */
  virtual void logDebug(const std::stringstream& ss) = 0;

  /**
   * @brief Method used to log general information
   *
   * @param ss String Stream containing the data to log
   */
  virtual void logInfo(const std::stringstream& ss) = 0;

  /**
   * @brief Method used to log Warnings
   *
   * @param ss String Stream containing the data to log
   */
  virtual void logWarning(const std::stringstream& ss) = 0;

  /**
   * @brief Method used to log Errors
   *
   * @param ss String Stream containing the data to log
   */
  virtual void logError(const std::stringstream& ss) = 0;

  /**
   * @brief Method used to log fatal conditions or events
   *
   * @param ss String Stream containing the data to log
   */
  virtual void logFatal(const std::stringstream& ss) = 0;
};

/**
 * @brief Fantasy name for a shared pointer object to a logger
 */
using LoggerPtr = std::shared_ptr<IIssueLogger>;
