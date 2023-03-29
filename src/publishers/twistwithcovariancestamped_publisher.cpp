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

#include <publishers/twistwithcovariancestamped_publisher.h>
#include <utils/utils.h>


TwistWithCovarianceStampedPublisher::TwistWithCovarianceStampedPublisher(sbp::State* state,
                                       const std::string_view topic_name,
                                       rclcpp::Node* node,
                                       const LoggerPtr& logger,
                                       const std::string& frame,
                                       const std::shared_ptr<Config>& config)
    : SBP2ROS2Publisher<geometry_msgs::msg::TwistWithCovarianceStamped,
                        sbp_msg_utc_time_t,
                        sbp_msg_vel_ned_cov_t>(state, topic_name, node, logger,
                                               frame, config) {}


void TwistWithCovarianceStampedPublisher::handle_sbp_msg(uint16_t sender_id,
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

void TwistWithCovarianceStampedPublisher::handle_sbp_msg(uint16_t sender_id,
                                        const sbp_msg_vel_ned_cov_t& msg) {
  if (0 == sender_id) return; // Ignore base station data

  if (SBP_VEL_NED_VELOCITY_MODE_INVALID !=
      SBP_VEL_NED_VELOCITY_MODE_GET(msg.flags)) {

    msg_.twist.twist.linear.x = static_cast<double>(msg.e) / 1e3;  // [m/s]
    msg_.twist.twist.linear.y = static_cast<double>(msg.n) / 1e3;  // [m/s]
    msg_.twist.twist.linear.z = -static_cast<double>(msg.d) / 1e3; // [m/s]

    msg_.twist.covariance[0] = msg.cov_e_e;
    msg_.twist.covariance[1] = msg.cov_n_e;
    msg_.twist.covariance[2] = -msg.cov_e_d;
    msg_.twist.covariance[6] = msg.cov_n_e;
    msg_.twist.covariance[7] = msg.cov_n_n;
    msg_.twist.covariance[8] = -msg.cov_n_d;
    msg_.twist.covariance[12] = -msg.cov_e_d;
    msg_.twist.covariance[13] = -msg.cov_n_d;
    msg_.twist.covariance[14] = msg.cov_d_d;
  }
  else {
    msg_.twist.covariance[0] = -1.0; // Twist is invalid
  }

  // Angular velocity is not provided
  msg_.twist.covariance[21] = -1.0;

  last_received_vel_ned_cov_tow_ = msg.tow;

  publish();
}

void TwistWithCovarianceStampedPublisher::publish() {
  if ((last_received_vel_ned_cov_tow_ == last_received_utc_time_tow_) ||
      !config_->getTimeStampSourceGNSS()) {
    if (0 == msg_.header.stamp.sec) {
      // Use current platform time if time from the GNSS receiver is not
      // available or if a local time source is selected
      msg_.header.stamp = node_->now();
    }

    msg_.header.frame_id = frame_;

    publisher_->publish(msg_);

    msg_ = geometry_msgs::msg::TwistWithCovarianceStamped();
    last_received_utc_time_tow_ = -1;
    last_received_vel_ned_cov_tow_ = -2;
  }
}
