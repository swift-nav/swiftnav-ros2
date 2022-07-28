#include <publishers/NavSatFixPublisher.h>
#include <iostream>

constexpr uint32_t MEASUREMENT_STATE = 0x00000001;
constexpr uint32_t POS_LLH_COV = 0x00000002;

NavSatFixPublisher::NavSatFixPublisher(sbp::State* state,
                                       const std::string& topic_name,
                                       rclcpp::Node* node, const bool enabled)
    : SBP2ROS2Publisher<sensor_msgs::msg::NavSatFix,
                        sbp_msg_measurement_state_t, sbp_msg_pos_llh_cov_t>(
          state, topic_name, node, enabled) {}

void NavSatFixPublisher::handle_sbp_msg(
    uint16_t sender_id, const sbp_msg_measurement_state_t& /*msg*/) {
  (void)sender_id;
  composition_mask_ |= MEASUREMENT_STATE;
  publish();
}

void NavSatFixPublisher::handle_sbp_msg(uint16_t sender_id,
                                        const sbp_msg_pos_llh_cov_t& msg) {
  (void)sender_id;
  msg_.latitude = msg.lat;
  composition_mask_ |= POS_LLH_COV;
  publish();
}

void NavSatFixPublisher::publish() {
  if (enabled_ && (composition_mask_ == MEASUREMENT_STATE + POS_LLH_COV)) {
    msg_.header.stamp = node_->now();
    publisher_->publish(msg_);
    composition_mask_ = 0U;
    msg_ = sensor_msgs::msg::NavSatFix();
  }
}