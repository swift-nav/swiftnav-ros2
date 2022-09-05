#include<test/mocked_logger.h>

#include<iostream>

void MockedLogger::logDebug(const std::stringstream& ss) {
  std::cout << "DEBUG->" << ss.str() << std::endl;
  last_logged_debug_ = ss.str();
}

void MockedLogger::logInfo(const std::stringstream& ss) {
  std::cout << "INFO->" << ss.str() << std::endl;
  last_logged_info_ = ss.str();
}

void MockedLogger::logWarning(const std::stringstream& ss) {
  std::cout << "WARN->" << ss.str() << std::endl;
  last_logged_warning_ = ss.str();
}

void MockedLogger::logError(const std::stringstream& ss) {
  std::cout << "ERROR->" << ss.str() << std::endl;
  last_logged_error_ = ss.str();
}
  
void MockedLogger::logFatal(const std::stringstream& ss) {
  std::cout << "FATAL->" << ss.str() << std::endl;
  last_logged_fatal_ = ss.str();
}

  std::string MockedLogger::getLastLoggedDebug() {
    return last_logged_debug_;
  }
  std::string MockedLogger::getLastLoggedInfo() {
    return last_logged_info_;
  }
  std::string MockedLogger::getLastLoggedWarning() {
    return last_logged_warning_;
  }
  std::string MockedLogger::getLastLoggedError() {
    return last_logged_error_;
  }
  std::string MockedLogger::getLastLoggedFatal() {
    return last_logged_fatal_;
  }

