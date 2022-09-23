#pragma once

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>

#include <string_view>

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
  virtual void logDebug(const std::string_view ss) = 0;

  /**
   * @brief Method used to log general information
   *
   * @param ss String Stream containing the data to log
   */
  virtual void logInfo(const std::string_view ss) = 0;

  /**
   * @brief Method used to log Warnings
   *
   * @param ss String Stream containing the data to log
   */
  virtual void logWarning(const std::string_view ss) = 0;

  /**
   * @brief Method used to log Errors
   *
   * @param ss String Stream containing the data to log
   */
  virtual void logError(const std::string_view ss) = 0;

  /**
   * @brief Method used to log fatal conditions or events
   *
   * @param ss String Stream containing the data to log
   */
  virtual void logFatal(const std::string_view ss) = 0;
};

/**
 * @brief Fantasy name for a shared pointer object to a logger
 */
using LoggerPtr = std::shared_ptr<IIssueLogger>;

// Logging macros
inline void LOG_FUNC(const LoggerPtr& logger, const int level,
                     const char* format, ...) {
  va_list args;

  if (!logger) return;

  va_start(args, format);
  const auto size = vsnprintf(nullptr, 0, format, args);
  if (size < 1) return;
  const uint32_t msg_len = static_cast<uint32_t>(size) + 1U;
  std::vector<char> msg(msg_len);
  vsnprintf(msg.data(), msg.size(), format, args);
  va_end(args);
  switch (level) {
    case 0:
      logger->logDebug(msg.data());
      break;

    case 1:
      logger->logInfo(msg.data());
      break;

    case 2:
      logger->logWarning(msg.data());
      break;

    case 3:
      logger->logError(msg.data());
      break;

    case 4:
      logger->logFatal(msg.data());
      break;

    default:
      break;
  }
}

#define LOG_DEBUG(logger, ...) LOG_FUNC(logger, 0, __VA_ARGS__)
#define LOG_INFO(logger, ...) LOG_FUNC(logger, 1, __VA_ARGS__)
#define LOG_WARN(logger, ...) LOG_FUNC(logger, 2, __VA_ARGS__)
#define LOG_ERROR(logger, ...) LOG_FUNC(logger, 3, __VA_ARGS__)
#define LOG_FATAL(logger, ...) LOG_FUNC(logger, 4, __VA_ARGS__)

// Contract checking macros
#define ASSERT_COND(cond, logger, str) \
  do {                                 \
    if (!(cond)) {                     \
      LOG_FATAL(logger, str);          \
      exit(1);                         \
    }                                  \
  } while (0)
