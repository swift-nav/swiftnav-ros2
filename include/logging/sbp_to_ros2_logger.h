#pragma once

#include <libsbp/cpp/message_handler.h>
#include <libsbp/cpp/state.h>
#include <logging/ros_logger.h>

/**
 * @brief Class to forward SBP messages to ROS 2 logging system
 */
class SBPToROS2Logger : private sbp::AllMessageHandler {
 public:
  SBPToROS2Logger() = delete;

  /**
   * @brief Construct a new SBPToROS2Logger object
   *
   * @param state SBP state object
   * @param logger ROS 2 logging object
   */
  SBPToROS2Logger(sbp::State* state, const LoggerPtr& logger)
      : sbp::AllMessageHandler(state), ros_logger_(logger) {}

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
};
