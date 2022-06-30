/*
 * Copyright (C) 2015-2023 Swift Navigation Inc.
 * Contact: https://support.swiftnav.com
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include<test/mocked_logger.h>

#include<iostream>

void MockedLogger::logDebug(const std::string_view ss) {
  std::cout << "DEBUG->" << ss << std::endl;
  last_logged_debug_ = ss;
}

void MockedLogger::logInfo(const std::string_view ss) {
  std::cout << "INFO->" << ss << std::endl;
  last_logged_info_ = ss;
}

void MockedLogger::logWarning(const std::string_view ss) {
  std::cout << "WARN->" << ss << std::endl;
  last_logged_warning_ = ss;
}

void MockedLogger::logError(const std::string_view ss) {
  std::cout << "ERROR->" << ss << std::endl;
  last_logged_error_ = ss;
}

void MockedLogger::logFatal(const std::string_view ss) {
  std::cout << "FATAL->" << ss << std::endl;
  last_logged_fatal_ = ss;
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

