#pragma once

#include "issue_logger.h"

class ROSLogger : public IIssueLogger {
 public:
  void logDebug(const std::stringstream& ss) override;
  void logInfo(const std::stringstream& ss) override;
  void logWarning(const std::stringstream& ss) override;
  void logError(const std::stringstream& ss) override;
  void logFatal(const std::stringstream& ss) override;
};
