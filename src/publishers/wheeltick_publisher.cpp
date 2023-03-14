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

#include <publishers/wheeltick_publisher.h>

WheeltickPublisher::WheeltickPublisher(sbp::State* state,
                                       const std::string& topic_name,
                                       rclcpp::Node* node,
                                       const LoggerPtr& logger,
                                       const std::string& frame)
    : SBP2ROS2Publisher<swiftnav_ros2_driver::msg::Wheeltick,
                        sbp_msg_wheeltick_t>(state, topic_name, node, logger,
                                             frame) {}

void WheeltickPublisher::handle_sbp_msg(uint16_t sender_id,
                                        const sbp_msg_wheeltick_t& msg) {
  (void)sender_id;

  msg_.source = msg.source;
  msg_.ticks = msg.ticks;
  msg_.time = msg.time;
  msg_.flags = msg.flags;
  publish();
}

void WheeltickPublisher::publish() {
  publisher_->publish(msg_);
  msg_ = swiftnav_ros2_driver::msg::Wheeltick();
}
