#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <libsbp/cpp/state.h>
#include <libsbp/cpp/message_handler.h>

#include <publishers/SBP2ROS2Publisher.h>

class NavSatFixPublisher : public SBP2ROS2Publisher<sensor_msgs::msg::NavSatFix, sbp_msg_gps_time_t, sbp_msg_pos_ecef_t> {
  public:

    NavSatFixPublisher(sbp::State *state, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> publisher, rclcpp::Node* node);

    void handle_sbp_msg(uint16_t sender_id, const sbp_msg_gps_time_t& msg);

    void handle_sbp_msg(uint16_t sender_id, const sbp_msg_pos_ecef_t& msg);

};