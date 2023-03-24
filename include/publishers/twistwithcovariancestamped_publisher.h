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
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <libsbp/cpp/message_handler.h>
#include <libsbp/cpp/state.h>

#include <publishers/base_publisher.h>
#include <publishers/sbp2ros2_publisher.h>

/**
 * @brief Class publishing ROS2 geometry_msgs::msg::TwistWithCovarianceStamped message.
 *
 */
class TwistWithCovarianceStampedPublisher
    : public BasePublisher,
      public SBP2ROS2Publisher<geometry_msgs::msg::TwistWithCovarianceStamped,
                               sbp_msg_utc_time_t,
                               sbp_msg_vel_ned_cov_t> {
 public:
  TwistWithCovarianceStampedPublisher() = delete;

  /**
   * @brief Construct a new TwistWithCovarianceStamped Publisher object
   *
   * @param state SBP State object
   * @param topic_name Name of the topic to publish a
   * geometry_msgs::msg::TwistWithCovarianceStamped message
   * @param node ROS 2 node object
   */
  TwistWithCovarianceStampedPublisher(sbp::State* state, const std::string_view topic_name,
                     rclcpp::Node* node, const LoggerPtr& logger,
                     const std::string& frame,
                     const std::shared_ptr<Config>& config);

  /**
   * @brief Handles a sbp_msg_utc_time_t message. It gets the
   * time stamp.
   *
   * @param sender_id Ignored
   * @param msg Incoming sbp_msg_utc_time_t
   */
  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_utc_time_t& msg);

  /**
   * @brief Handles a sbp_msg_vel_ned_cov_t message. It gets the velocities,
   * and covariance matrix.
   *
   * @param sender_id Ignored
   * @param msg Incoming sbp_msg_vel_ned_cov_t
   */
  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_vel_ned_cov_t& msg);

 protected:
  /**
   * @brief Checks that the ROS2 geometry_msgs::msg::TwistWithCovarianceStamped is complete, if so,
   * it publishes it
   *
   */
  void publish() override;

 private:
  int32_t last_received_utc_time_tow_{-1};
  int32_t last_received_vel_ned_cov_tow_{-2};

};
