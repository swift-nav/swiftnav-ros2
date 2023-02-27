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


#define GPS_EPOCH           315964800u
#define GPS_MINUS_UTC_SECS         18u  // Since 2017-01-01


TimeReferencePublisher::TimeReferencePublisher(sbp::State* state,
                                               const std::string& topic_name,
                                               rclcpp::Node* node,
                                               const LoggerPtr& logger,
                                               const std::string& frame)
    : SBP2ROS2Publisher<sensor_msgs::msg::TimeReference, sbp_msg_gps_time_t>(
          state, topic_name, node, logger, frame) {}


void TimeReferencePublisher::handle_sbp_msg(uint16_t sender_id,
                                            const sbp_msg_gps_time_t& msg) {
  (void)sender_id;

  msg_ = sensor_msgs::msg::TimeReference();

  if ( SBP_GPS_TIME_TIME_SOURCE_NONE != (msg.flags & SBP_GPS_TIME_TIME_SOURCE_MASK) ) {

    unsigned int  s = msg.wn * 604800 + msg.tow / 1000u;
    unsigned int ns = ((msg.tow % 1000u) * 1000000u) + msg.ns_residual;

    // Linux time
    msg_.header.stamp.sec     = GPS_EPOCH - GPS_MINUS_UTC_SECS + s;
    msg_.header.stamp.nanosec = ns;

    // GPS time
    msg_.time_ref.sec         = s;
    msg_.time_ref.nanosec     = ns;
  }
  else {
    msg_.time_ref.sec = -1;
  }

  publish();
}


void TimeReferencePublisher::publish() {

  if ( 0 == msg_.header.stamp.sec ) {
    msg_.header.stamp = node_->now();
  }

  msg_.source = frame_;

  publisher_->publish(msg_);
}
