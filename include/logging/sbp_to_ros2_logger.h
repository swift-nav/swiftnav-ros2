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

#include <libsbp/cpp/message_handler.h>
#include <libsbp/cpp/state.h>
#include <logging/ros_logger.h>
#include <logging/sbp_file_logger.h>

#include <memory>

/**
 * @brief Class to forward SBP messages to ROS 2 logging system. It also creates
 * an SBP dump file
 */
class SBPToROS2Logger : private sbp::AllMessageHandler {
 public:
  SBPToROS2Logger() = delete;

  /**
   * @brief Construct a new SBPToROS2Logger object
   *
   * @param state SBP state object
   * @param logger ROS 2 logging object
   * @param log_messages Flag that enables/disables SBP file logging
   * @param log_path Path where to put the log file
   */
  SBPToROS2Logger(sbp::State* state, const LoggerPtr& logger,
                  const bool log_messages, const std::string& log_path);

  /**
   * @brief Callback function for processiing SBP messages
   *
   * @param sender_id Sender ID
   * @param msg_type Type of message
   * @param msg General SBP message structure
   */
  void handle_sbp_message(uint16_t sender_id, sbp_msg_type_t msg_type,
                          const sbp_msg_t& msg);

 private:
  LoggerPtr ros_logger_; /** @brief ROS logging object */
  std::unique_ptr<SbpFileLogger>
      file_logger_; /** @brief SBP file logger object */
};
