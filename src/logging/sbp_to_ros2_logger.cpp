#include <logging/sbp_to_ros2_logger.h>

void SBPToROS2Logger::handle_sbp_message(uint16_t sender_id,
                                         sbp_msg_type_t msg_type,
                                         const sbp_msg_t& msg) {
  (void)sender_id;
  if (msg_type == SBP_MSG_LOG) {
    switch (msg.log.level) {
      case SBP_LOG_LOGGING_LEVEL_WARN:
        LOG_WARN(ros_logger_, "SBP(WARN): " << msg.log.text.data);
        break;

      case SBP_LOG_LOGGING_LEVEL_EMERG:
        LOG_FATAL(ros_logger_, "SBP(EMERG): " << msg.log.text.data);
        break;

      case SBP_LOG_LOGGING_LEVEL_ALERT:
        LOG_FATAL(ros_logger_, "SBP(ALERT): " << msg.log.text.data);
        break;

      case SBP_LOG_LOGGING_LEVEL_CRIT:
        LOG_ERROR(ros_logger_, "SBP(CRIT): " << msg.log.text.data);
        break;

      case SBP_LOG_LOGGING_LEVEL_ERROR:
        LOG_ERROR(ros_logger_, "SBP(ERROR): " << msg.log.text.data);
        break;
    }
  }
}
