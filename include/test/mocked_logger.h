#pragma once

#include<logging/issue_logger.h>

// *******************************************
// Dummy console implementation of a Logger
class MockedLogger : public IIssueLogger {
 public:
  void logDebug(const std::stringstream& ss) override;

  void logInfo(const std::stringstream& ss) override;

  void logWarning(const std::stringstream& ss) override;

  void logError(const std::stringstream& ss) override;

  void logFatal(const std::stringstream& ss) override;
  
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