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

#pragma once

#include <gps_msgs/msg/gps_fix.hpp>
#include <rclcpp/rclcpp.hpp>

#include <libsbp/cpp/message_handler.h>
#include <libsbp/cpp/state.h>

#include <publishers/dummy_publisher.h>
#include <publishers/sbp2ros2_publisher.h>

/**
 * @brief Class that listens for sbp_msg_obs_t and sbp_msg_pos_llh_cov_t,
 * publishing a gps_comon::msg::GPSFix ros2 message.
 *
 */
class GPSFixPublisher
    : public DummyPublisher,
      public SBP2ROS2Publisher<
          gps_msgs::msg::GPSFix, sbp_msg_pos_llh_acc_t, sbp_msg_pos_llh_cov_t,
          sbp_msg_vel_cog_t, sbp_msg_vel_ned_cov_t, sbp_msg_orient_euler_t,
          sbp_msg_dops_t, sbp_msg_gps_time_t, sbp_msg_obs_t> {
 public:
  GPSFixPublisher() = delete;

  /**
   * @brief Construct a new Gps Fix Publisher object
   *
   * @param state SBP State object
   * @param topic_name Name of the topic to publish a gps_msgs::msg::GPSFix
   * message
   * @param node ROS 2 node object
   * @param enabled Flag telling if the topic should be published (true) or not
   * (false)
   */
  GPSFixPublisher(sbp::State* state, const std::string& topic_name,
                  rclcpp::Node* node, const LoggerPtr& logger,
                  const std::string& frame);

  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_pos_llh_acc_t& msg);

  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_pos_llh_cov_t& msg);

  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_vel_cog_t& msg);

  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_vel_ned_cov_t& msg);

  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_orient_euler_t& msg);

  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_dops_t& msg);

  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_gps_time_t& msg);

  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_obs_t& msg);

 protected:
  /**
   * @brief Checks that the Ros2 gps_msgs::msg::GPSFix is complete, if so,
   * it publishes it
   *
   */
  void publish() override;

 private:
  /**
   * @brief Loads the covariance matrix values in the ROS2 message from values
   * in the sbp message. To do so it converts the covariance matrix from NED to
   * ENU.
   *
   * @param msg sbp_msg_pos_llh_cov_t
   */
  void loadCovarianceMatrix(const sbp_msg_pos_llh_cov_t& msg);

  bool ok_to_publish(const u32& tow) const;

  u32 last_received_pos_llh_cov_tow_{0};
  u32 last_received_vel_cog_tow_{0};
  u32 last_received_vel_ned_cov_tow_{0};
  u32 last_received_orient_euler_tow_{0};
  u32 last_received_dops_tow_{0};
  u32 last_received_gps_time_tow_{0};
  u32 last_received_obs_tow_{0};

  static constexpr uint32_t MAX_POS_LLH_COV_TIME_DIFF = 2000;
  static constexpr uint32_t MAX_VEL_COG_TIME_DIFF = 2000;
  static constexpr uint32_t MAX_VEL_NED_COV_TIME_DIFF = 2000;
  static constexpr uint32_t MAX_ORIENT_EULER_TIME_DIFF = 2000;
  static constexpr uint32_t MAX_DOPS_TIME_DIFF = 2000;
  static constexpr uint32_t MAX_GPS_TIME_TIME_DIFF = 2000;
  static constexpr uint32_t MAX_OBS_TIME_DIFF_MS = 2000;
};