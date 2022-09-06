#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <string_view>
#include "issue_logger.h"

class ROSLogger : public IIssueLogger {
 public:
  ROSLogger() = delete;
  explicit ROSLogger(const int64_t log_delay);
  void logDebug(const std::string_view ss) override;
  void logInfo(const std::string_view ss) override;
  void logWarning(const std::string_view ss) override;
  void logError(const std::string_view ss) override;
  void logFatal(const std::string_view ss) override;

 private:
  /**
   * @brief Method to determine if we could output a log to ROS or not
   *
   * @param output_str String to output
   * @return true We are allowed to output the string
   * @return false We're not allowed to output the string
   */
  bool canLog(const std::string_view output_str) const;

  /**
   * @brief This method updates the information about the last message sent to
   * ROS log
   *
   * @param output_str Las string sent to ROS
   */
  void updateLogStatus(const std::string_view output_str);

  std::string last_output_str_; /** @brief Last string sent to ROS */
  std::chrono::time_point<std::chrono::system_clock>
      last_output_time_; /** @brief Time of the last message sent to ROS */
  int64_t timeout_{0LL}; /** @brief Time that must pass before we're allowed to
                            send the same string */
};
