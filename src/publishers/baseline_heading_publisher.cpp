#include <publishers/baseline_heading_publisher.h>

BaselineHeadingPublisher::BaselineHeadingPublisher(
    sbp::State* state, const std::string& topic_name, rclcpp::Node* node,
    const LoggerPtr& logger, const std::string& frame)
    : SBP2ROS2Publisher<swiftnav_ros2_driver::msg::BaselineHeading,
                        sbp_msg_baseline_heading_t>(state, topic_name, node,
                                                    logger, frame) {}

void BaselineHeadingPublisher::handle_sbp_msg(
    uint16_t sender_id, const sbp_msg_baseline_heading_t& msg) {
  (void)sender_id;

  msg_.tow = msg.tow;
  msg_.heading = msg.heading;
  msg_.n_sats = msg.n_sats;
  msg_.flags = msg.flags;
  publish();
}

void BaselineHeadingPublisher::publish() {
  publisher_->publish(msg_);
  msg_ = swiftnav_ros2_driver::msg::BaselineHeading();
}
