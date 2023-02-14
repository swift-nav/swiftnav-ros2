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

#include <publishers/angular_rate_publisher.h>

AngularRatePublisher::AngularRatePublisher(sbp::State* state,
                                           const std::string& topic_name,
                                           rclcpp::Node* node,
                                           const LoggerPtr& logger,
                                           const std::string& frame)
    : SBP2ROS2Publisher<swiftnav_ros2_driver::msg::AngularRate,
                        sbp_msg_angular_rate_t>(state, topic_name, node, logger,
                                                frame) {}

void AngularRatePublisher::handle_sbp_msg(uint16_t sender_id,
                                          const sbp_msg_angular_rate_t& msg) {
  (void)sender_id;

  msg_.tow = msg.tow;
  msg_.x = msg.x;
  msg_.y = msg.y;
  msg_.z = msg.z;
  msg_.flags = msg.flags;
  publish();
}

void AngularRatePublisher::publish() {
  publisher_->publish(msg_);
  msg_ = swiftnav_ros2_driver::msg::AngularRate();
}
