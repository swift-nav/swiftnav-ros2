#include <publishers/wheeltick_publisher.h>

WheeltickPublisher::WheeltickPublisher(
    sbp::State* state, const std::string& topic_name, rclcpp::Node* node,
    const LoggerPtr& logger, const bool enabled, const std::string& frame)
    : SBP2ROS2Publisher<swiftnav_ros2_driver::msg::Wheeltick,
                        sbp_msg_wheeltick_t>(state, topic_name, node, logger,
                                             enabled, frame) {}

void WheeltickPublisher::handle_sbp_msg(uint16_t sender_id,
                                        const sbp_msg_wheeltick_t& msg) {
  (void)sender_id;

  msg_.source = msg.source;
  msg_.ticks = msg.ticks;
  msg_.time = msg.time;
  msg_.flags = msg.flags;
  publish();
}

void WheeltickPublisher::publish() {
  if (enabled_) {
    publisher_->publish(msg_);
    msg_ = swiftnav_ros2_driver::msg::Wheeltick();
  }
}
