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

#include <subscribers/odometry_subscriber.h>

OdometrySubscriber::OdometrySubscriber(rclcpp::Node* node, sbp::State* state,
                                       const std::string& topic_name,
                                       const LoggerPtr& logger)
    : ROS22SBPSubscriber(node, state, logger),
      subscriber_(node_->create_subscription<nav_msgs::msg::Odometry>(
          topic_name, 10,
          std::bind(&OdometrySubscriber::topic_callback, this, _1))) {}

void OdometrySubscriber::topic_callback(const nav_msgs::msg::Odometry& msg) {
  sbp_msg_t sbp_msg;
  sbp_msg.odometry.tow = msg.header.stamp.sec * 1000;
  sbp_msg.odometry.velocity =
      static_cast<s32>(std::round(msg.twist.twist.linear.x * 1000.0));
  sbp_msg.odometry.flags = 2;

  send_message(SbpMsgOdometry, sbp_msg);
}