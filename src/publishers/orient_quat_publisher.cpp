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

#include <publishers/orient_quat_publisher.h>

OrientQuatPublisher::OrientQuatPublisher(sbp::State* state,
                                         const std::string& topic_name,
                                         rclcpp::Node* node,
                                         const LoggerPtr& logger,
                                         const std::string& frame)
    : SBP2ROS2Publisher<swiftnav_ros2_driver::msg::OrientQuat,
                        sbp_msg_orient_quat_t>(state, topic_name, node, logger,
                                               frame) {}

void OrientQuatPublisher::handle_sbp_msg(uint16_t sender_id,
                                         const sbp_msg_orient_quat_t& msg) {
  (void)sender_id;

  msg_.tow = msg.tow;
  msg_.w = msg.w;
  msg_.w_accuracy = msg.w_accuracy;
  msg_.x = msg.x;
  msg_.x_accuracy = msg.x_accuracy;
  msg_.y = msg.y;
  msg_.y_accuracy = msg.y_accuracy;
  msg_.z = msg.z;
  msg_.z_accuracy = msg.z_accuracy;
  msg_.flags = msg.flags;
  publish();
}

void OrientQuatPublisher::publish() {
  publisher_->publish(msg_);
  msg_ = swiftnav_ros2_driver::msg::OrientQuat();
}
