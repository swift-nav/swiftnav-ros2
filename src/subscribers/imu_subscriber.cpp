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

#include <subscribers/imu_subscriber.h>
#include <cmath>

IMUSubscriber::IMUSubscriber(rclcpp::Node* node, sbp::State* state,
                             const std::string& topic_name,
                             const LoggerPtr& logger)
    : ROS22SBPSubscriber(node, state, logger),
      subscriber_(node_->create_subscription<sensor_msgs::msg::Imu>(
          topic_name, 10,
          std::bind(&IMUSubscriber::topic_callback, this, _1))) {}

void IMUSubscriber::topic_callback(const sensor_msgs::msg::Imu& msg) {
  sbp_msg_t sbp_msg;

  const uint32_t ms_since_epoch =
      (msg.header.stamp.sec * 1000) + (msg.header.stamp.nanosec / 1000000U);

  // I'll use here "Reference is Unknown" because PC epoch is not in the
  // available options
  sbp_msg.imu_raw.tow = 0b10000000000000000000000000000000;

  // TODO: ***** This calculation should be confirmed
  sbp_msg.imu_raw.tow |= ms_since_epoch / 256U;
  sbp_msg.imu_raw.tow_f = static_cast<u8>(ms_since_epoch % 256U);
  // TODO: ***** This calculation should be confirmed

  sbp_msg.imu_raw.acc_x =
      static_cast<s16>(std::round(msg.linear_acceleration.x));
  sbp_msg.imu_raw.acc_y =
      static_cast<s16>(std::round(msg.linear_acceleration.y));
  sbp_msg.imu_raw.acc_z =
      static_cast<s16>(std::round(msg.linear_acceleration.z));
  sbp_msg.imu_raw.gyr_x = static_cast<s16>(std::round(msg.angular_velocity.x));
  sbp_msg.imu_raw.gyr_y = static_cast<s16>(std::round(msg.angular_velocity.y));
  sbp_msg.imu_raw.gyr_z = static_cast<s16>(std::round(msg.angular_velocity.z));

  send_message(SbpMsgImuRaw, sbp_msg);
}
