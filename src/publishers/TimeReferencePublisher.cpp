#include <publishers/TimeReferencePublisher.h>

TimeReferencePublisher::TimeReferencePublisher(sbp::State *state, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::TimeReference>> publisher, rclcpp::Node* node) 
  : SBP2ROS2Publisher<sensor_msgs::msg::TimeReference, sbp_msg_gps_time_t>(state, publisher, node) {
}

void TimeReferencePublisher::handle_sbp_msg(uint16_t sender_id, const sbp_msg_gps_time_t& msg) {
 (void)sender_id;

 auto message = sensor_msgs::msg::TimeReference();
 message.header.stamp = node_->now();
 message.time_ref.sec = msg.tow/1000;
 message.time_ref.nanosec = msg.ns_residual;
 message.source = "gps";

 publisher_->publish(message);
}
