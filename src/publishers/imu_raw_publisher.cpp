#include <publishers/imu_raw_publisher.h>

ImuRawPublisher::ImuRawPublisher(sbp::State* state,
                                 const std::string& topic_name,
                                 rclcpp::Node* node, const LoggerPtr& logger,
                                 const bool enabled, const std::string& frame)
    : SBP2ROS2Publisher<swiftnav_ros2_driver::msg::ImuRaw, sbp_msg_imu_raw_t>(
          state, topic_name, node, logger, enabled, frame) {}

void ImuRawPublisher::handle_sbp_msg(uint16_t sender_id,
                                     const sbp_msg_imu_raw_t& msg) {
  (void)sender_id;

  msg_.acc_x = msg.acc_x;
  msg_.acc_y = msg.acc_y;
  msg_.acc_z = msg.acc_z;
  msg_.gyr_x = msg.gyr_x;
  msg_.gyr_y = msg.gyr_y;
  msg_.gyr_z = msg.gyr_z;
  msg_.tow = msg.tow;
  msg_.tow_f = msg.tow_f;
  publish();
}

void ImuRawPublisher::publish() {
  if (enabled_) {
    publisher_->publish(msg_);
    msg_ = swiftnav_ros2_driver::msg::ImuRaw();
  }
}
