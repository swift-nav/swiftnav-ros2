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


static constexpr uint32_t TOW_MS   = 1000u;
static constexpr uint32_t MS_TO_NS = 1000000u;


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
    msg_.time_ref.sec     =   msg.tow / TOW_MS;
    msg_.time_ref.nanosec = ((msg.tow % TOW_MS) * MS_TO_NS) + msg.ns_residual;
  }
  else {
    msg_.time_ref.sec = -1;
  }

  publish();
}


void TimeReferencePublisher::publish() {
  msg_.header.stamp = node_->now();
  msg_.source = frame_;
  publisher_->publish(msg_);
}
