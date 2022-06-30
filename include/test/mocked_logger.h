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
