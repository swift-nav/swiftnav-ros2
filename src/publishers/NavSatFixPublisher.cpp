#include <publishers/NavSatFixPublisher.h>
#include <iostream>

NavSatFixPublisher::NavSatFixPublisher(
    sbp::State* state,
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> publisher,
    rclcpp::Node* node)
    : SBP2ROS2Publisher<sensor_msgs::msg::NavSatFix,
                        sbp_msg_measurement_state_t, sbp_msg_pos_llh_t>(
          state, publisher, node) {}

void NavSatFixPublisher::handle_sbp_msg(
    uint16_t sender_id, const sbp_msg_measurement_state_t& /*msg*/) {
  (void)sender_id;
}

void NavSatFixPublisher::handle_sbp_msg(uint16_t sender_id,
                                        const sbp_msg_pos_llh_t& msg) {
  (void)sender_id;
  auto message = sensor_msgs::msg::NavSatFix();
  message.header.stamp = node_->now();
  message.latitude = msg.lat;
  std::cout << "NavSatFix->Lat: " << msg.lat << std::endl;
  publisher_->publish(message);
}