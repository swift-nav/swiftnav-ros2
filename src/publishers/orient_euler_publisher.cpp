#include <publishers/orient_euler_publisher.h>

OrientEulerPublisher::OrientEulerPublisher(sbp::State* state,
                                           const std::string& topic_name,
                                           rclcpp::Node* node,
                                           const LoggerPtr& logger,
                                           const std::string& frame)
    : SBP2ROS2Publisher<swiftnav_ros2_driver::msg::OrientEuler,
                        sbp_msg_orient_euler_t>(state, topic_name, node, logger,
                                                frame) {}

void OrientEulerPublisher::handle_sbp_msg(uint16_t sender_id,
                                          const sbp_msg_orient_euler_t& msg) {
  (void)sender_id;

  msg_.tow = msg.tow;
  msg_.pitch = msg.pitch;
  msg_.pitch_accuracy = msg.pitch_accuracy;
  msg_.roll = msg.roll;
  msg_.roll_accuracy = msg.roll_accuracy;
  msg_.yaw = msg.yaw;
  msg_.yaw_accuracy = msg.yaw_accuracy;
  msg_.flags = msg.flags;
  publish();
}

void OrientEulerPublisher::publish() {
  publisher_->publish(msg_);
  msg_ = swiftnav_ros2_driver::msg::OrientEuler();
}
