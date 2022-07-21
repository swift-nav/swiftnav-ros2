#include <publishers/NavSatFixPublisher.h>

NavSatFixPublisher::NavSatFixPublisher(sbp::State *state, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> publisher, rclcpp::Node * node) 
  : SBP2ROS2Publisher<sensor_msgs::msg::NavSatFix, sbp_msg_gps_time_t, sbp_msg_pos_ecef_t>(state, publisher, node) {
}

void NavSatFixPublisher::handle_sbp_msg(uint16_t sender_id, const sbp_msg_gps_time_t& msg) {
 (void)sender_id;
}

void NavSatFixPublisher::handle_sbp_msg(uint16_t sender_id, const sbp_msg_pos_ecef_t& msg) {
  (void)sender_id;
  auto message = sensor_msgs::msg::NavSatFix();
  publisher_->publish(message);
}