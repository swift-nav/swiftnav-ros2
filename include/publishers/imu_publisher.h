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

#include <publishers/dummy_publisher.h>
#include <publishers/sbp2ros2_publisher.h>

/**
 * @brief Class publishing ROS2 sensor_msgs::msg::Imu message.
 *
 */
class ImuPublisher
    : public DummyPublisher,
      public SBP2ROS2Publisher<sensor_msgs::msg::Imu,
                               sbp_msg_utc_time_t,
                               sbp_msg_gps_time_t,
                               sbp_msg_gnss_time_offset_t,
                               sbp_msg_imu_aux_t,
                               sbp_msg_imu_raw_t
                              > {
 public:
  ImuPublisher() = delete;
  ImuPublisher(sbp::State* state, const std::string& topic_name,
                         rclcpp::Node* node, const LoggerPtr& logger,
                         const std::string& frame,
                         const std::shared_ptr<Config>& config);

  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_utc_time_t& msg);
  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_gps_time_t& msg);
  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_gnss_time_offset_t& msg);
  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_imu_aux_t& msg);
  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_imu_raw_t& msg);

 protected:
  void publish() override;

 private:

  void compute_utc_offset( void );

  uint32_t last_received_utc_time_tow = -1;
  uint32_t last_received_gps_time_tow = -2;

  double linux_stamp_s = 0.0;
  double gps_stamp_s   = 0.0;

  double utc_offset_s = 0.0;
  double gnss_time_offset_s = 0.0;

  double acc_res_mps2 = 0.0;
  double gyro_res_rad = 0.0;

};
