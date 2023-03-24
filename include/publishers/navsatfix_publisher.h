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
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <libsbp/cpp/message_handler.h>
#include <libsbp/cpp/state.h>

#include <publishers/base_publisher.h>
#include <publishers/sbp2ros2_publisher.h>

/**
 * @brief Class publishing ROS2 sensor_msgs::msg::NavSatFix message.
 *
 */
class NavSatFixPublisher
    : public BasePublisher,
      public SBP2ROS2Publisher<sensor_msgs::msg::NavSatFix,
                               sbp_msg_measurement_state_t, sbp_msg_utc_time_t,
                               sbp_msg_pos_llh_cov_t> {
 public:
  NavSatFixPublisher() = delete;

  /**
   * @brief Construct a new Nav Sat Fix Publisher object
   *
   * @param state SBP State object
   * @param topic_name Name of the topic to publish a
   * sensor_msgs::msg::NavSatFix message
   * @param node ROS 2 node object
   */
  NavSatFixPublisher(sbp::State* state, const std::string_view topic_name,
                     rclcpp::Node* node, const LoggerPtr& logger,
                     const std::string& frame,
                     const std::shared_ptr<Config>& config);

  /**
   * @brief Handles a sbp_msg_measurement_state_t message. It gets the
   * constellation for each satellite in the measurement states.
   *
   * @param sender_id Ignored
   * @param msg Incoming sbp_msg_measurement_state_t
   */
  void handle_sbp_msg(uint16_t sender_id,
                      const sbp_msg_measurement_state_t& msg);

  /**
   * @brief Handles a sbp_msg_utc_time_t message. It gets the
   * time stamp.
   *
   * @param sender_id Ignored
   * @param msg Incoming sbp_msg_utc_time_t
   */
  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_utc_time_t& msg);

  /**
   * @brief Handles a sbp_msg_pos_llh_cov_t message. It gets the position mode,
   * latitude, longitude, altitude and covariance matrix.
   *
   * @param sender_id Ignored
   * @param msg Incoming sbp_msg_pos_llh_cov_t
   */
  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_pos_llh_cov_t& msg);

 protected:
  /**
   * @brief Checks that the ROS2 sensor_msgs::msg::NavSatFix is complete, if so,
   * it publishes it
   *
   */
  void publish() override;

 private:
  uint32_t last_received_utc_time_tow = -1;
  uint32_t last_received_pos_llh_cov_tow = -2;

  uint16_t status_service;
};
