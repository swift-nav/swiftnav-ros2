#pragma once

#include <rclcpp/rclcpp.hpp>
#include <swiftnav_ros2_driver/msg/orient_euler.hpp>

#include <libsbp/cpp/message_handler.h>
#include <libsbp/cpp/state.h>

#include <publishers/SBP2ROS2Publisher.h>

class OrientEulerPublisher
    : public SBP2ROS2Publisher<swiftnav_ros2_driver::msg::OrientEuler,
                               sbp_msg_orient_euler_t> {
 public:
  OrientEulerPublisher() = delete;
  OrientEulerPublisher(sbp::State* state, const std::string& topic_name,
                       rclcpp::Node* node, const LoggerPtr& logger,
                       const bool enabled, const std::string& frame);

  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_orient_euler_t& msg);

 protected:
  void publish() override;
};