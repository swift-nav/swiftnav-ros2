#pragma once

#include <rclcpp/rclcpp.hpp>
#include <swiftnav_ros2_driver/msg/baseline_heading.hpp>

#include <libsbp/cpp/message_handler.h>
#include <libsbp/cpp/state.h>

#include <publishers/SBP2ROS2Publisher.h>
#include <publishers/dummy_publisher.h>

class BaselineHeadingPublisher
    : public DummyPublisher,
      public SBP2ROS2Publisher<swiftnav_ros2_driver::msg::BaselineHeading,
                               sbp_msg_baseline_heading_t> {
 public:
  BaselineHeadingPublisher() = delete;
  BaselineHeadingPublisher(sbp::State* state, const std::string& topic_name,
                           rclcpp::Node* node, const LoggerPtr& logger,
                           const std::string& frame);

  void handle_sbp_msg(uint16_t sender_id,
                      const sbp_msg_baseline_heading_t& msg);

 protected:
  void publish() override;
};