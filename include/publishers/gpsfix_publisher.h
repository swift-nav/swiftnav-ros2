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

#include <gps_msgs/msg/gps_fix.hpp>
#include <rclcpp/rclcpp.hpp>

#include <libsbp/cpp/message_handler.h>
#include <libsbp/cpp/state.h>

#include <publishers/base_publisher.h>
#include <publishers/sbp2ros2_publisher.h>

/**
 * @brief Class publishing a gps_comon::msg::GPSFix ROS2 message.
 *
 */
class GPSFixPublisher
    : public BasePublisher,
      public SBP2ROS2Publisher<gps_msgs::msg::GPSFix, sbp_msg_gps_time_t,
                               sbp_msg_utc_time_t, sbp_msg_pos_llh_cov_t,
                               sbp_msg_vel_ned_cov_t, sbp_msg_orient_euler_t,
                               sbp_msg_dops_t> {
 public:
  GPSFixPublisher() = delete;

  /**
   * @brief Construct a new GPS Fix Publisher object
   *
   * @param state SBP State object
   * @param topic_name Name of the topic to publish a gps_msgs::msg::GPSFix
   * message
   * @param node ROS 2 node object
   */
  GPSFixPublisher(sbp::State* state, const std::string_view topic_name,
                  rclcpp::Node* node, const LoggerPtr& logger,
                  const std::string& frame,
                  const std::shared_ptr<Config>& config);

  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_gps_time_t& msg);
  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_utc_time_t& msg);
  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_pos_llh_cov_t& msg);
  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_vel_ned_cov_t& msg);
  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_orient_euler_t& msg);
  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_dops_t& msg);

 protected:
  /**
   * @brief Checks that the ROS2 gps_msgs::msg::GPSFix is complete, if so,
   * it publishes it
   *
   */
  void publish() override;

 private:
  uint32_t last_received_gps_time_tow = -1;
  uint32_t last_received_utc_time_tow = -2;
  uint32_t last_received_pos_llh_cov_tow = -3;
  uint32_t last_received_vel_ned_cov_tow = -4;
  uint32_t last_received_orient_euler_tow = -5;

  bool orientation_present = false;

  bool vel_ned_track_valid = false;
  double vel_ned_track_deg = 0.0;
  double vel_ned_err_track_deg = 0.0;

  bool orientation_track_valid = false;
  double orientation_track_deg = 0.0;
  double orientation_err_track_deg = 0.0;

  bool last_track_valid = false;
  double last_track_deg = 0.0;
  double last_err_track_deg = 0.0;

  time_t dops_time_s;
  double gdop = 0.0;
  double pdop = 0.0;
  double hdop = 0.0;
  double vdop = 0.0;
  double tdop = 0.0;
};
