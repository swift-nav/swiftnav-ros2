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

#include <publishers/odometry_publisher.h>

OdometryPublisher::OdometryPublisher(sbp::State* state,
                                     const std::string& topic_name,
                                     rclcpp::Node* node,
                                     const LoggerPtr& logger,
                                     const std::string& frame)
    : SBP2ROS2Publisher<swiftnav_ros2_driver::msg::Odometry,
                        sbp_msg_odometry_t>(state, topic_name, node, logger,
                                            frame) {}

void OdometryPublisher::handle_sbp_msg(uint16_t sender_id,
                                       const sbp_msg_odometry_t& msg) {
  (void)sender_id;

  msg_.tow = msg.tow;
  msg_.velocity = msg.velocity;
  msg_.flags = msg.flags;
  publish();
}

void OdometryPublisher::publish() {
  publisher_->publish(msg_);
  msg_ = swiftnav_ros2_driver::msg::Odometry();
}
