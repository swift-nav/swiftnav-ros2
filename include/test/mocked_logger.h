#pragma once

#include<logging/issue_logger.h>
#include <string>

// *******************************************
// Dummy console implementation of a Logger
class MockedLogger : public IIssueLogger {
 public:
  void logDebug(const std::string_view ss) override;

  void logInfo(const std::string_view ss) override;

  void logWarning(const std::string_view ss) override;

  void logError(const std::string_view ss) override;

  void logFatal(const std::string_view ss) override;

  std::string getLastLoggedDebug();
  std::string getLastLoggedInfo();
  std::string getLastLoggedWarning();
  std::string getLastLoggedError();
  std::string getLastLoggedFatal();

private:
    std::string last_logged_debug_;
    std::string last_logged_info_;
    std::string last_logged_warning_;
    std::string last_logged_error_;
    std::string last_logged_fatal_;

};