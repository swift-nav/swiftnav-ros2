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

#include <publishers/baseline_heading_publisher.h>


//!! Temporary here
//--------------------------------------------------------

// Move to config file
static bool timestamp_source_gnss = true;
static double baseline_dir_offset_deg = 0.0;
static double baseline_dip_offset_deg = 0.0;

// Move to utils.h
extern time_t Utils_UtcToLinuxTime( unsigned int year, unsigned int month, unsigned int day,
                             unsigned int hours, unsigned int minutes, unsigned int seconds );




BaselineHeadingPublisher::BaselineHeadingPublisher(
    sbp::State* state, const std::string& topic_name, rclcpp::Node* node,
    const LoggerPtr& logger, const std::string& frame)
    : SBP2ROS2Publisher<swiftnav_ros2_driver::msg::BaselineHeading,
                        sbp_msg_utc_time_t, sbp_msg_baseline_ned_t>(state, topic_name, node, logger, frame) {}


void BaselineHeadingPublisher::handle_sbp_msg( uint16_t sender_id,
                                               const sbp_msg_utc_time_t& msg ) {
  (void)sender_id;

  if ( timestamp_source_gnss ) {
    if ( SBP_UTC_TIME_TIME_SOURCE_NONE != SBP_UTC_TIME_TIME_SOURCE_GET(msg.flags) ) {

//      msg_.header.stamp.sec     = Utils_UtcToLinuxTime( msg.year, msg.month, msg.day, msg.hours, msg.minutes, msg.seconds );
//      msg_.header.stamp.nanosec = msg.ns;
    }

    last_received_utc_time_tow = msg.tow;

    publish();
  }
}


void BaselineHeadingPublisher::handle_sbp_msg( uint16_t sender_id,
                                               const sbp_msg_baseline_ned_t& msg) {
  (void)sender_id;

  msg_.mode = SBP_BASELINE_NED_FIX_MODE_GET( msg.flags );

  if ( SBP_BASELINE_NED_FIX_MODE_INVALID != msg_.mode ) {

    msg_.satellites_used = msg.n_sats;

    msg_.baseline_n_m = (double)msg.n / 1e3;
    msg_.baseline_e_m = (double)msg.e / 1e3;
    msg_.baseline_d_m = (double)msg.d / 1e3;

    msg_.baseline_err_h_m = (double)msg.h_accuracy / 1e3;
    msg_.baseline_err_v_m = (double)msg.v_accuracy / 1e3;

    double b_m = msg_.baseline_n_m * msg_.baseline_n_m + msg_.baseline_e_m * msg_.baseline_e_m;
    msg_.baseline_length_m   = sqrt( b_m + msg_.baseline_d_m * msg_.baseline_d_m );
    msg_.baseline_length_h_m = sqrt( b_m );

    if ( SBP_BASELINE_NED_FIX_MODE_FIXED_RTK == SBP_BASELINE_NED_FIX_MODE_GET(msg.flags) ) {

      // Baseline Direction (bearing or heading)
      double dir_rad = atan2( msg_.baseline_e_m, msg_.baseline_n_m );
      if ( dir_rad < 0.0 ) {
        dir_rad += 2.0 * M_PI;
      }
      msg_.baseline_dir_deg = dir_rad * 180.0 / M_PI + baseline_dir_offset_deg;  // [deg]
      if ( msg_.baseline_dir_deg < 0.0 ) {
        msg_.baseline_dir_deg += 360.0;
      }
      else if ( msg_.baseline_dir_deg >= 360.0 ) {
        msg_.baseline_dir_deg -= 360.0;
      }
      msg_.baseline_dir_err_deg = -1;  //!! TODO

      // RTK Dip
      double dip_rad = atan2( msg_.baseline_d_m, msg_.baseline_length_h_m );
      msg_.baseline_dip_deg = dip_rad * 180.0 / M_PI + baseline_dip_offset_deg;  // [deg]

      msg_.baseline_dip_err_deg = -1;  //!! TODO

      msg_.baseline_orientation_valid = true;
    }
  }

  last_received_baseline_ned_tow = msg.tow;

  publish();
}


void BaselineHeadingPublisher::publish() {

  if ( (last_received_baseline_ned_tow == last_received_utc_time_tow) || !timestamp_source_gnss ) {
#if 0
    if ( 0 == msg_.header.stamp.sec ) {
      // Use current platform time if time from the GNSS receiver is not available or if a local time source is selected
      msg_.header.stamp = node_->now();
    }

    msg_.header.frame_id = frame_;
#endif
    publisher_->publish(msg_);

    msg_ = swiftnav_ros2_driver::msg::BaselineHeading();
    last_received_utc_time_tow     = -1;
    last_received_baseline_ned_tow = -2;
  }
}
