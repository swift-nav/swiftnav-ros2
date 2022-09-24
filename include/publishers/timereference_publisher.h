#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

#include <libsbp/cpp/message_handler.h>
#include <libsbp/cpp/state.h>

#include <publishers/dummy_publisher.h>
#include <publishers/sbp2ros2_publisher.h>

class TimeReferencePublisher
    : public DummyPublisher,
      public SBP2ROS2Publisher<sensor_msgs::msg::TimeReference,
                               sbp_msg_gps_time_t> {
 public:
  TimeReferencePublisher() = delete;
  TimeReferencePublisher(sbp::State* state, const std::string& topic_name,
                         rclcpp::Node* node, const LoggerPtr& logger,
                         const std::string& frame);

  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_gps_time_t& msg);

 protected:
  void publish() override;
};