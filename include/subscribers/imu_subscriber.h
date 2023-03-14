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
#include <sensor_msgs/msg/imu.hpp>

#include <subscribers/dummy_subscriber.h>
#include <subscribers/ros2_2_sbp_subscriber.h>

class IMUSubscriber : public DummySubscriber, public ROS22SBPSubscriber {
 public:
  IMUSubscriber() = delete;

  IMUSubscriber(rclcpp::Node* node, sbp::State* state,
                const std::string& topic_name, const LoggerPtr& logger);

 protected:
  virtual void topic_callback(const sensor_msgs::msg::Imu& msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr
      subscriber_; /** @brief ROS 2 subscriber */
};
