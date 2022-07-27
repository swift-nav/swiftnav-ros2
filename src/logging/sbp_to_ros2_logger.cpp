#include <logging/sbp_to_ros2_logger.h>

void SBPToROS2Logger::handle_sbp_message(uint16_t sender_id,
                                         sbp_msg_type_t msg_type,
                                         const sbp_msg_t& msg) {
  (void)sender_id;
  if (msg_type == SBP_MSG_LOG) {
    switch (msg.log.level) {
      case SBP_LOG_LOGGING_LEVEL_WARN:
        LOG_WARN(ros_logger_, msg.log.text.data);
        break;

      case SBP_LOG_LOGGING_LEVEL_EMERG:
      case SBP_LOG_LOGGING_LEVEL_ALERT:
        LOG_FATAL(ros_logger_, msg.log.text.data);
        break;

      case SBP_LOG_LOGGING_LEVEL_CRIT:
      case SBP_LOG_LOGGING_LEVEL_ERROR:
        LOG_ERROR(ros_logger_, msg.log.text.data);
        break;
    }
  }
}
