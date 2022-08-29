#include <publishers/TimeReferencePublisher.h>

static constexpr uint32_t TOW_MS = 1000U;
static constexpr uint32_t MS_TO_NS = 1000000;

TimeReferencePublisher::TimeReferencePublisher(sbp::State* state,
                                               const std::string& topic_name,
                                               rclcpp::Node* node,
                                               const bool enabled, 
                                               const std::string& frame)
    : SBP2ROS2Publisher<sensor_msgs::msg::TimeReference, sbp_msg_gps_time_t>(
          state, topic_name, node, enabled, frame) {}

void TimeReferencePublisher::handle_sbp_msg(uint16_t sender_id,
                                            const sbp_msg_gps_time_t& msg) {
  (void)sender_id;
  (void)msg;

  msg_.time_ref.sec = msg.tow / TOW_MS;
  msg_.time_ref.nanosec = ((msg.tow % TOW_MS) * MS_TO_NS) + msg.ns_residual;
  publish();
}

void TimeReferencePublisher::publish() {
  if (enabled_) {
    msg_.header.stamp = node_->now();
    msg_.source = frame_;
    publisher_->publish(msg_);
    msg_ = sensor_msgs::msg::TimeReference();
  }
}
