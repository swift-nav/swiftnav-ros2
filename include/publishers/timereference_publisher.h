/*
 * Copyright (C) 2010-2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

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