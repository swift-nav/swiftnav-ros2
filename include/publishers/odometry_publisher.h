#pragma once

#include <rclcpp/rclcpp.hpp>
#include <swiftnav_ros2_driver/msg/odometry.hpp>

#include <libsbp/cpp/message_handler.h>
#include <libsbp/cpp/state.h>

#include <publishers/SBP2ROS2Publisher.h>

class OdometryPublisher
    : public SBP2ROS2Publisher<swiftnav_ros2_driver::msg::Odometry,
                               sbp_msg_odometry_t> {
 public:
  OdometryPublisher() = delete;
  OdometryPublisher(sbp::State* state, const std::string& topic_name,
                    rclcpp::Node* node, const LoggerPtr& logger,
                    const bool enabled, const std::string& frame);

  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_odometry_t& msg);

 protected:
  void publish() override;
};
