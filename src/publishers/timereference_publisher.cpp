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

// Temporary here
extern time_t Utils_UtcToLinuxTime( unsigned int year, unsigned int month, unsigned int day,
                             unsigned int hours, unsigned int minutes, unsigned int seconds );


TimeReferencePublisher::TimeReferencePublisher(sbp::State* state,
                                               const std::string& topic_name,
                                               rclcpp::Node* node,
                                               const LoggerPtr& logger,
                                               const std::string& frame)
    : SBP2ROS2Publisher<sensor_msgs::msg::TimeReference,
                        sbp_msg_utc_time_t,
                        sbp_msg_gps_time_t>(state, topic_name, node, logger, frame) {}


void TimeReferencePublisher::handle_sbp_msg( uint16_t sender_id,
                                             const sbp_msg_utc_time_t& msg ) {
  (void)sender_id;

  if ( SBP_UTC_TIME_TIME_SOURCE_NONE != (msg.flags & SBP_UTC_TIME_TIME_SOURCE_MASK) ) {

    msg_.header.stamp.sec     = Utils_UtcToLinuxTime( msg.year, msg.month, msg.day,
                                                      msg.hours, msg.minutes, msg.seconds );
    msg_.header.stamp.nanosec = msg.ns;
  }

  last_received_utc_time_tow = msg.tow;

  publish();
}


void TimeReferencePublisher::handle_sbp_msg(uint16_t sender_id,
                                            const sbp_msg_gps_time_t& msg) {
  (void)sender_id;

  if ( SBP_GPS_TIME_TIME_SOURCE_NONE != (msg.flags & SBP_GPS_TIME_TIME_SOURCE_MASK) ) {

    msg_.time_ref.sec     = msg.wn * 604800u + msg.tow / 1000u;
    msg_.time_ref.nanosec = ((msg.tow % 1000u) * 1000000u) + msg.ns_residual;
  }
  else {
    msg_.time_ref.sec = -1;
  }

  last_received_gps_time_tow = msg.tow;

  publish();
}


void TimeReferencePublisher::publish() {

  if ( last_received_utc_time_tow == last_received_gps_time_tow ) {

    if ( 0 == msg_.header.stamp.sec ) {
      msg_.header.stamp = node_->now();
    }

    msg_.source = frame_;

    publisher_->publish(msg_);

    msg_ = sensor_msgs::msg::TimeReference();
    last_received_utc_time_tow = -1;
    last_received_gps_time_tow = -2;
  }
}
