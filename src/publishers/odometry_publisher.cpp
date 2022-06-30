#include <publishers/odometry_publisher.h>

OdometryPublisher::OdometryPublisher(sbp::State* state,
                                     const std::string& topic_name,
                                     rclcpp::Node* node,
                                     const LoggerPtr& logger,
                                     const std::string& frame)
    : SBP2ROS2Publisher<swiftnav_ros2_driver::msg::Odometry,
                        sbp_msg_odometry_t>(state, topic_name, node, logger,
                                            frame) {}

void OdometryPublisher::handle_sbp_msg(uint16_t sender_id,
                                       const sbp_msg_odometry_t& msg) {
  (void)sender_id;

  msg_.tow = msg.tow;
  msg_.velocity = msg.velocity;
  msg_.flags = msg.flags;
  publish();
}

void OdometryPublisher::publish() {
  publisher_->publish(msg_);
  msg_ = swiftnav_ros2_driver::msg::Odometry();
}
