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

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <subscribers/dummy_subscriber.h>
#include <subscribers/ros2_2_sbp_subscriber.h>

class OdometrySubscriber : public DummySubscriber, public ROS22SBPSubscriber {
 public:
  OdometrySubscriber() = delete;

  OdometrySubscriber(rclcpp::Node* node, sbp::State* state,
                     const std::string& topic_name, const LoggerPtr& logger);

 protected:
  virtual void topic_callback(const nav_msgs::msg::Odometry& msg);

 private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      subscriber_; /** @brief ROS 2 subscriber */
};
