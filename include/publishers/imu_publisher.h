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

#include <libsbp/cpp/message_handler.h>
#include <libsbp/cpp/state.h>

#include <publishers/base_publisher.h>
#include <publishers/sbp2ros2_publisher.h>

/**
 * @brief Class publishing ROS2 sensor_msgs::msg::Imu message.
 *
 */
class ImuPublisher
    : public BasePublisher,
      public SBP2ROS2Publisher<sensor_msgs::msg::Imu, sbp_msg_utc_time_t,
                               sbp_msg_gps_time_t, sbp_msg_gnss_time_offset_t,
                               sbp_msg_imu_aux_t, sbp_msg_imu_raw_t> {
 public:
  ImuPublisher() = delete;
  ImuPublisher(sbp::State* state, const std::string_view topic_name,
               rclcpp::Node* node, const LoggerPtr& logger,
               const std::string& frame, const std::shared_ptr<Config>& config);

  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_utc_time_t& msg);
  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_gps_time_t& msg);
  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_gnss_time_offset_t& msg);
  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_imu_aux_t& msg);
  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_imu_raw_t& msg);

 protected:
  void publish() override;

 private:
  void compute_utc_offset();

  int32_t last_received_utc_time_tow_{-1};
  int32_t last_received_gps_time_tow_{-2};

  double linux_stamp_s_{0.0};
  double gps_stamp_s_{0.0};

  bool gps_week_valid_{false};
  uint32_t gps_week_{0U};

  bool utc_offset_valid_{false};
  double utc_offset_s_{0.0};

  bool gps_time_offset_valid_{false};
  double gps_time_offset_s_{0.0};

  uint32_t last_gps_week_{0U};
  uint32_t last_imu_raw_tow_ms_{0U};

  enum stamp_source { STAMP_SOURCE_DEFAULT, STAMP_SOURCE_PLATFORM, STAMP_SOURCE_GNSS };
  uint32_t stamp_source_{STAMP_SOURCE_DEFAULT};
  uint32_t last_stamp_source_{STAMP_SOURCE_DEFAULT};

  double acc_res_mps2_{0.0};
  double gyro_res_rad_{0.0};
};
