#include <publishers/angular_rate_publisher.h>

AngularRatePublisher::AngularRatePublisher(
    sbp::State* state, const std::string& topic_name, rclcpp::Node* node,
    const LoggerPtr& logger, const bool enabled, const std::string& frame)
    : SBP2ROS2Publisher<swiftnav_ros2_driver::msg::AngularRate,
                        sbp_msg_angular_rate_t>(state, topic_name, node, logger,
                                                enabled, frame) {}

void AngularRatePublisher::handle_sbp_msg(uint16_t sender_id,
                                          const sbp_msg_angular_rate_t& msg) {
  (void)sender_id;

  msg_.tow = msg.tow;
  msg_.x = msg.x;
  msg_.y = msg.y;
  msg_.z = msg.z;
  msg_.flags = msg.flags;
  publish();
}

void AngularRatePublisher::publish() {
  if (enabled_) {
    publisher_->publish(msg_);
    msg_ = swiftnav_ros2_driver::msg::AngularRate();
  }
}
