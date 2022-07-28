#include <publishers/TimeReferencePublisher.h>

TimeReferencePublisher::TimeReferencePublisher(sbp::State* state,
                                               const std::string& topic_name,
                                               rclcpp::Node* node,
                                               const bool enabled)
    : SBP2ROS2Publisher<sensor_msgs::msg::TimeReference, sbp_msg_gps_time_t>(
          state, topic_name, node, enabled) {}

void TimeReferencePublisher::handle_sbp_msg(uint16_t sender_id, const sbp_msg_gps_time_t& msg) {
 (void)sender_id;

 msg_.time_ref.sec = msg.tow / 1000;
 msg_.time_ref.nanosec = msg.ns_residual;
 publish();
}

void TimeReferencePublisher::publish() {
  if (enabled_) {
    msg_.header.stamp = node_->now();
    msg_.source = "gps";
    publisher_->publish(msg_);
    msg_ = sensor_msgs::msg::TimeReference();
  }
}
