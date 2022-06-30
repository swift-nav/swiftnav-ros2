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

#include <publishers/baseline_publisher.h>
#include <utils/utils.h>

BaselinePublisher::BaselinePublisher(sbp::State* state,
                                     const std::string_view topic_name,
                                     rclcpp::Node* node,
                                     const LoggerPtr& logger,
                                     const std::string& frame,
                                     const std::shared_ptr<Config>& config)
    : SBP2ROS2Publisher<swiftnav_ros2_driver::msg::Baseline, sbp_msg_utc_time_t,
                        sbp_msg_baseline_ned_t>(state, topic_name, node, logger,
                                                frame, config) {}

void BaselinePublisher::handle_sbp_msg(uint16_t sender_id,
                                       const sbp_msg_utc_time_t& msg) {
  if (0 == sender_id) return; // Ignore base station data

  if (config_->getTimeStampSourceGNSS()) {
    if (SBP_UTC_TIME_TIME_SOURCE_NONE !=
        SBP_UTC_TIME_TIME_SOURCE_GET(msg.flags)) {
      struct tm utc;

      utc.tm_year = msg.year;
      utc.tm_mon = msg.month;
      utc.tm_mday = msg.day;
      utc.tm_hour = msg.hours;
      utc.tm_min = msg.minutes;
      utc.tm_sec = msg.seconds;
      msg_.header.stamp.sec = TimeUtils::utcToLinuxTime(utc);
      msg_.header.stamp.nanosec = msg.ns;
    }

    last_received_utc_time_tow_ = msg.tow;

    publish();
  }
}

void BaselinePublisher::handle_sbp_msg(uint16_t sender_id,
                                       const sbp_msg_baseline_ned_t& msg) {
  if (0 == sender_id) return; // Ignore base station data

  msg_.mode = SBP_BASELINE_NED_FIX_MODE_GET(msg.flags);

  if (SBP_BASELINE_NED_FIX_MODE_INVALID != msg_.mode) {
    msg_.satellites_used = msg.n_sats;

    msg_.baseline_n_m = static_cast<double>(msg.n) / 1e3;
    msg_.baseline_e_m = static_cast<double>(msg.e) / 1e3;
    msg_.baseline_d_m = static_cast<double>(msg.d) / 1e3;

    msg_.baseline_err_h_m = static_cast<double>(msg.h_accuracy) / 1e3;
    msg_.baseline_err_v_m = static_cast<double>(msg.v_accuracy) / 1e3;

    double b_m = msg_.baseline_n_m * msg_.baseline_n_m +
                 msg_.baseline_e_m * msg_.baseline_e_m;
    msg_.baseline_length_m = sqrt(b_m + msg_.baseline_d_m * msg_.baseline_d_m);
    msg_.baseline_length_h_m = sqrt(b_m);

    if (SBP_BASELINE_NED_FIX_MODE_FIXED_RTK ==
        SBP_BASELINE_NED_FIX_MODE_GET(msg.flags)) {
      // Baseline Direction (bearing or heading)
      double dir_rad = atan2(msg_.baseline_e_m, msg_.baseline_n_m);
      if (dir_rad < 0.0) {
        dir_rad += 2.0 * M_PI;
      }
      msg_.baseline_dir_deg =
          dir_rad * 180.0 / M_PI + config_->getBaseLineDirOffsetDeg();  // [deg]
      if (msg_.baseline_dir_deg < 0.0) {
        msg_.baseline_dir_deg += 360.0;
      } else if (msg_.baseline_dir_deg >= 360.0) {
        msg_.baseline_dir_deg -= 360.0;
      }
      msg_.baseline_dir_err_deg =
          atan2(msg_.baseline_err_h_m, msg_.baseline_length_h_m) * 180.0 /
          M_PI;

      // Baseline Dip
      double dip_rad = atan2(msg_.baseline_d_m, msg_.baseline_length_h_m);
      msg_.baseline_dip_deg =
          dip_rad * 180.0 / M_PI + config_->getBaseLineDipOffsetDeg();  // [deg]

      msg_.baseline_dip_err_deg =
          atan2(msg_.baseline_err_v_m, msg_.baseline_length_h_m) * 180.0 /
          M_PI;

      msg_.baseline_orientation_valid = true;
    }
  }

  last_received_baseline_ned_tow_ = msg.tow;

  publish();
}

void BaselinePublisher::publish() {
  if ((last_received_baseline_ned_tow_ == last_received_utc_time_tow_) ||
      !config_->getTimeStampSourceGNSS()) {
    if (0 == msg_.header.stamp.sec) {
      // Use current platform time if time from the GNSS receiver is not
      // available or if a local time source is selected
      msg_.header.stamp = node_->now();
    }

    msg_.header.frame_id = frame_;
    publisher_->publish(msg_);

    msg_ = swiftnav_ros2_driver::msg::Baseline();
    last_received_utc_time_tow_ = -1;
    last_received_baseline_ned_tow_ = -2;
  }
}
