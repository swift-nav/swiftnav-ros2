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

#include <logging/sbp_to_ros2_logger.h>

SBPToROS2Logger::SBPToROS2Logger(sbp::State* state, const LoggerPtr& logger,
                                 const bool log_messages,
                                 const std::string& log_path)
    : sbp::AllMessageHandler(state), ros_logger_(logger) {
  if (log_messages)
    file_logger_ = std::make_unique<SbpFileLogger>(log_path, logger);
}

void SBPToROS2Logger::handle_sbp_message(uint16_t sender_id,
                                         sbp_msg_type_t msg_type,
                                         const sbp_msg_t& msg) {
  (void)sender_id;
  if (file_logger_) file_logger_->insert(msg_type, msg);

  if (msg_type == SBP_MSG_LOG) {
    switch (msg.log.level) {
      case SBP_LOG_LOGGING_LEVEL_WARN:
        LOG_WARN(ros_logger_, "SBP(WARN): %s", msg.log.text.data);
        break;

      case SBP_LOG_LOGGING_LEVEL_EMERG:
        LOG_FATAL(ros_logger_, "SBP(EMERG): %s", msg.log.text.data);
        break;

      case SBP_LOG_LOGGING_LEVEL_ALERT:
        LOG_FATAL(ros_logger_, "SBP(ALERT): %s", msg.log.text.data);
        break;

      case SBP_LOG_LOGGING_LEVEL_CRIT:
        LOG_ERROR(ros_logger_, "SBP(CRIT): %s", msg.log.text.data);
        break;

      case SBP_LOG_LOGGING_LEVEL_ERROR:
        LOG_ERROR(ros_logger_, "SBP(ERROR): %s", msg.log.text.data);
        break;
    }
  }
}
