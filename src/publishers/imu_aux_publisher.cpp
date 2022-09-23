#include <publishers/imu_aux_publisher.h>

ImuAuxPublisher::ImuAuxPublisher(sbp::State* state,
                                 const std::string& topic_name,
                                 rclcpp::Node* node, const LoggerPtr& logger,
                                 const bool enabled, const std::string& frame)
    : SBP2ROS2Publisher<swiftnav_ros2_driver::msg::ImuAux, sbp_msg_imu_aux_t>(
          state, topic_name, node, logger, enabled, frame) {}

void ImuAuxPublisher::handle_sbp_msg(uint16_t sender_id,
                                     const sbp_msg_imu_aux_t& msg) {
  (void)sender_id;

  msg_.imu_conf = msg.imu_conf;
  msg_.imu_type = msg.imu_type;
  msg_.temp = msg.temp;
  publish();
}

void ImuAuxPublisher::publish() {
  if (enabled_) {
    publisher_->publish(msg_);
    msg_ = swiftnav_ros2_driver::msg::ImuAux();
  }
}
