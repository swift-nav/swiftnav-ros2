#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <libsbp/cpp/state.h>
#include <libsbp/cpp/message_handler.h>

#include <publishers/SBP2ROS2Publisher.h>

/**
 * @brief Class that merges SBP messages into ROS 2 NavSatFix
 */
class NavSatFixPublisher : public SBP2ROS2Publisher<sensor_msgs::msg::NavSatFix,
                                                    sbp_msg_measurement_state_t,
                                                    sbp_msg_pos_llh_cov_t> {
 public:
  NavSatFixPublisher() = delete;
  NavSatFixPublisher(sbp::State* state, const std::string& topic_name,
                     rclcpp::Node* node, const bool enabled);

  void handle_sbp_msg(uint16_t sender_id,
                      const sbp_msg_measurement_state_t& msg);

  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_pos_llh_cov_t& msg);

 protected:
  void publish() override;
};