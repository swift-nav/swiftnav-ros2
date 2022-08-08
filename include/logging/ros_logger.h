#pragma once

#include <chrono>
#include <cstdint>
#include <string_view>
#include "issue_logger.h"

class ROSLogger : public IIssueLogger {
 public:
  explicit ROSLogger(const int64_t log_delay);
  void logDebug(const std::stringstream& ss) override;
  void logInfo(const std::stringstream& ss) override;
  void logWarning(const std::stringstream& ss) override;
  void logError(const std::stringstream& ss) override;
  void logFatal(const std::stringstream& ss) override;

 private:
  bool canLog(const std::string_view output_str) const;
  void updateLogStatus(const std::string_view output_str);

  std::string last_output_str_;
  std::chrono::time_point<std::chrono::system_clock> last_output_time_;
  int64_t timeout_{0LL};
};
