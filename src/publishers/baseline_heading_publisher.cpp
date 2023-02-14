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

#include <publishers/baseline_heading_publisher.h>

BaselineHeadingPublisher::BaselineHeadingPublisher(
    sbp::State* state, const std::string& topic_name, rclcpp::Node* node,
    const LoggerPtr& logger, const std::string& frame)
    : SBP2ROS2Publisher<swiftnav_ros2_driver::msg::BaselineHeading,
                        sbp_msg_baseline_heading_t>(state, topic_name, node,
                                                    logger, frame) {}

void BaselineHeadingPublisher::handle_sbp_msg(
    uint16_t sender_id, const sbp_msg_baseline_heading_t& msg) {
  (void)sender_id;

  msg_.tow = msg.tow;
  msg_.heading = msg.heading;
  msg_.n_sats = msg.n_sats;
  msg_.flags = msg.flags;
  publish();
}

void BaselineHeadingPublisher::publish() {
  publisher_->publish(msg_);
  msg_ = swiftnav_ros2_driver::msg::BaselineHeading();
}
