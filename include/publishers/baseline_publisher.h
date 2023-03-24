/*
 * Copyright (C) 2015-2023 Swift Navigation Inc.
 * Contact: https://support.swiftnav.com
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
#include <swiftnav_ros2_driver/msg/baseline.hpp>

#include <libsbp/cpp/message_handler.h>
#include <libsbp/cpp/state.h>

#include <publishers/base_publisher.h>
#include <publishers/sbp2ros2_publisher.h>

class BaselinePublisher
    : public BasePublisher,
      public SBP2ROS2Publisher<swiftnav_ros2_driver::msg::Baseline,
                               sbp_msg_utc_time_t, sbp_msg_baseline_ned_t> {
 public:
  BaselinePublisher() = delete;
  BaselinePublisher(sbp::State* state, const std::string_view topic_name,
                    rclcpp::Node* node, const LoggerPtr& logger,
                    const std::string& frame,
                    const std::shared_ptr<Config>& config);

  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_utc_time_t& msg);
  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_baseline_ned_t& msg);

 protected:
  void publish() override;

 private:
  int32_t last_received_utc_time_tow_{-1};
  int32_t last_received_baseline_ned_tow_{-2};
};
