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

#include <publishers/timereference_publisher.h>
#include <utils/utils.h>

TimeReferencePublisher::TimeReferencePublisher(
    sbp::State* state, const std::string_view topic_name, rclcpp::Node* node,
    const LoggerPtr& logger, const std::string& frame,
    const std::shared_ptr<Config>& config)
    : SBP2ROS2Publisher<sensor_msgs::msg::TimeReference, sbp_msg_utc_time_t,
                        sbp_msg_gps_time_t>(state, topic_name, node, logger,
                                            frame, config) {}

void TimeReferencePublisher::handle_sbp_msg(uint16_t sender_id,
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

void TimeReferencePublisher::handle_sbp_msg(uint16_t sender_id,
                                            const sbp_msg_gps_time_t& msg) {
  if (0 == sender_id) return; // Ignore base station data

  if (SBP_GPS_TIME_TIME_SOURCE_NONE !=
      SBP_GPS_TIME_TIME_SOURCE_GET(msg.flags)) {
    msg_.time_ref.sec = msg.wn * 604800u + msg.tow / 1000u;
    msg_.time_ref.nanosec = ((msg.tow % 1000u) * 1000000u) + msg.ns_residual;
  } else {
    msg_.time_ref.sec = -1;
  }

  last_received_gps_time_tow_ = msg.tow;

  publish();
}

void TimeReferencePublisher::publish() {
  if ((last_received_gps_time_tow_ == last_received_utc_time_tow_) ||
      !config_->getTimeStampSourceGNSS()) {
    if (0 == msg_.header.stamp.sec) {
      // Use current platform time if time from the GNSS receiver is not
      // available or if a local time source is selected
      msg_.header.stamp = node_->now();
    }

    msg_.source = frame_;

    publisher_->publish(msg_);

    msg_ = sensor_msgs::msg::TimeReference();
    last_received_utc_time_tow_ = -1;
    last_received_gps_time_tow_ = -2;
  }
}
