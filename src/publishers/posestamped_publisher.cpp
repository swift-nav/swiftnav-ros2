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

#include <publishers/posestamped_publisher.h>
#include <cmath>

PoseStampedPublisher::PoseStampedPublisher(sbp::State* state,
                                           const std::string& topic_name,
                                           rclcpp::Node* node,
                                           const LoggerPtr& logger,
                                           const std::string& frame)
    : SBP2ROS2Publisher<geometry_msgs::msg::PoseStamped, sbp_msg_pos_ecef_t,
                        sbp_msg_orient_euler_t>(state, topic_name, node, logger,
                                                frame) {}

void PoseStampedPublisher::handle_sbp_msg(uint16_t sender_id,
                                          const sbp_msg_pos_ecef_t& msg) {
  (void)sender_id;

  // OBS msg has not arrived yet.
  if (last_received_orient_euler_tow_ == 0) {
    LOG_WARN(logger_,
             "Orient euler message has not arrived yet. Not publishing");
    return;
  }

  // Last received OBS msg tow is too old
  const u32 time_diff = (last_received_orient_euler_tow_ > msg.tow)
                            ? last_received_orient_euler_tow_ - msg.tow
                            : msg.tow - last_received_orient_euler_tow_;
  if (time_diff > MAX_ORIENT_EULER_TIME_DIFF_) {
    LOG_WARN(logger_,
             "Time difference between ORIENT EULER message and POS ECEF "
             "message is larger than Max");
    return;
  }

  msg_.pose.position.x = msg.x;
  msg_.pose.position.y = msg.y;
  msg_.pose.position.z = msg.z;

  publish();
}

void PoseStampedPublisher::handle_sbp_msg(uint16_t sender_id,
                                          const sbp_msg_orient_euler_t& msg) {
  (void)sender_id;

  quat_.setEuler(this->microdegreesToRads(msg.pitch),
                 this->microdegreesToRads(msg.yaw),
                 this->microdegreesToRads(msg.roll));

  msg_.pose.orientation.x = quat_.getX();
  msg_.pose.orientation.y = quat_.getY();
  msg_.pose.orientation.z = quat_.getZ();
  msg_.pose.orientation.w = quat_.getW();

  last_received_orient_euler_tow_ = msg.tow;
}

void PoseStampedPublisher::publish() {
  msg_.header.stamp = node_->now();
  msg_.header.frame_id = frame_;
  publisher_->publish(msg_);
}

tf2Scalar PoseStampedPublisher::microdegreesToRads(
    const int32_t microdegrees) const {
  return microdegrees * M_PI / 1000000 * 180.0;
}