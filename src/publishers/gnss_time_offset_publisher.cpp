#include <publishers/gnss_time_offset_publisher.h>

GnssTimeOffsetPublisher::GnssTimeOffsetPublisher(
    sbp::State* state, const std::string& topic_name, rclcpp::Node* node,
    const LoggerPtr& logger, const bool enabled, const std::string& frame)
    : SBP2ROS2Publisher<swiftnav_ros2_driver::msg::GnssTimeOffset,
                        sbp_msg_gnss_time_offset_t>(state, topic_name, node,
                                                    logger, enabled, frame) {}

void GnssTimeOffsetPublisher::handle_sbp_msg(
    uint16_t sender_id, const sbp_msg_gnss_time_offset_t& msg) {
  (void)sender_id;

  msg_.weeks = msg.weeks;
  msg_.milliseconds = msg.milliseconds;
  msg_.microseconds = msg.microseconds;
  msg_.flags = msg.flags;
  publish();
}

void GnssTimeOffsetPublisher::publish() {
  if (enabled_) {
    publisher_->publish(msg_);
    msg_ = swiftnav_ros2_driver::msg::GnssTimeOffset();
  }
}
