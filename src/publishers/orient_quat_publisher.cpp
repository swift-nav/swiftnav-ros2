#include <publishers/orient_quat_publisher.h>

OrientQuatPublisher::OrientQuatPublisher(
    sbp::State* state, const std::string& topic_name, rclcpp::Node* node,
    const LoggerPtr& logger, const bool enabled, const std::string& frame)
    : SBP2ROS2Publisher<swiftnav_ros2_driver::msg::OrientQuat,
                        sbp_msg_orient_quat_t>(state, topic_name, node, logger,
                                               enabled, frame) {}

void OrientQuatPublisher::handle_sbp_msg(uint16_t sender_id,
                                         const sbp_msg_orient_quat_t& msg) {
  (void)sender_id;

  msg_.tow = msg.tow;
  msg_.w = msg.w;
  msg_.w_accuracy = msg.w_accuracy;
  msg_.x = msg.x;
  msg_.x_accuracy = msg.x_accuracy;
  msg_.y = msg.y;
  msg_.y_accuracy = msg.y_accuracy;
  msg_.z = msg.z;
  msg_.z_accuracy = msg.z_accuracy;
  msg_.flags = msg.flags;
  publish();
}

void OrientQuatPublisher::publish() {
  if (enabled_) {
    publisher_->publish(msg_);
    msg_ = swiftnav_ros2_driver::msg::OrientQuat();
  }
}