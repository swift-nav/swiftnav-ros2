#pragma once

#include <rclcpp/rclcpp.hpp>
#include <swiftnav_ros2_driver/msg/wheeltick.hpp>

#include <libsbp/cpp/message_handler.h>
#include <libsbp/cpp/state.h>

#include <publishers/SBP2ROS2Publisher.h>

class WheeltickPublisher
    : public SBP2ROS2Publisher<swiftnav_ros2_driver::msg::Wheeltick,
                               sbp_msg_wheeltick_t> {
 public:
  WheeltickPublisher() = delete;
  WheeltickPublisher(sbp::State* state, const std::string& topic_name,
                     rclcpp::Node* node, const LoggerPtr& logger,
                     const bool enabled, const std::string& frame);

  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_wheeltick_t& msg);

 protected:
  void publish() override;
};
