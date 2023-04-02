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

#include <publishers/gpsfix_publisher.h>
#include <utils/utils.h>

GPSFixPublisher::GPSFixPublisher(sbp::State* state,
                                 const std::string_view topic_name,
                                 rclcpp::Node* node, const LoggerPtr& logger,
                                 const std::string& frame,
                                 const std::shared_ptr<Config>& config)
    : SBP2ROS2Publisher<gps_msgs::msg::GPSFix, sbp_msg_gps_time_t,
                        sbp_msg_utc_time_t, sbp_msg_pos_llh_cov_t,
                        sbp_msg_vel_ned_cov_t, sbp_msg_orient_euler_t,
                        sbp_msg_dops_t>(state, topic_name, node, logger, frame,
                                        config) {}

void GPSFixPublisher::handle_sbp_msg(uint16_t sender_id,
                                     const sbp_msg_gps_time_t& msg) {
  if (0 == sender_id) return; // Ignore base station data

  if (SBP_GPS_TIME_TIME_SOURCE_NONE !=
      SBP_GPS_TIME_TIME_SOURCE_GET(msg.flags)) {
    msg_.time = (double)(msg.wn) * 604800.0 + (double)msg.tow / 1e3 +
                (double)msg.ns_residual / 1e9;  // [s]
  }

  last_received_gps_time_tow = msg.tow;

  publish();
}

void GPSFixPublisher::handle_sbp_msg(uint16_t sender_id,
                                     const sbp_msg_utc_time_t& msg) {
  if (0 == sender_id) return; // Ignore base station data

  if (config_->getTimeStampSourceGNSS()) {
    // Use GNSS receiver reported time to stamp the data
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

    last_received_utc_time_tow = msg.tow;

    publish();
  }
}

void GPSFixPublisher::handle_sbp_msg(uint16_t sender_id,
                                     const sbp_msg_pos_llh_cov_t& msg) {
  if (0 == sender_id) return; // Ignore base station data

  switch (SBP_POS_LLH_FIX_MODE_GET(msg.flags)) {
    case SBP_POS_LLH_FIX_MODE_SINGLE_POINT_POSITION:
      msg_.status.status = gps_msgs::msg::GPSStatus::STATUS_FIX;
      break;
    case SBP_POS_LLH_FIX_MODE_DIFFERENTIAL_GNSS:
      msg_.status.status = gps_msgs::msg::GPSStatus::STATUS_DGPS_FIX;
      break;
    case SBP_POS_LLH_FIX_MODE_FLOAT_RTK:
      msg_.status.status = gps_msgs::msg::GPSStatus::STATUS_GBAS_FIX;
      break;
    case SBP_POS_LLH_FIX_MODE_FIXED_RTK:
      msg_.status.status = gps_msgs::msg::GPSStatus::STATUS_GBAS_FIX;
      break;
    case SBP_POS_LLH_FIX_MODE_DEAD_RECKONING:
      msg_.status.status = gps_msgs::msg::GPSStatus::STATUS_FIX;
      break;
    case SBP_POS_LLH_FIX_MODE_SBAS_POSITION:
      msg_.status.status = gps_msgs::msg::GPSStatus::STATUS_SBAS_FIX;
      break;
    default:
      msg_.status.status = gps_msgs::msg::GPSStatus::STATUS_NO_FIX;
  }  // switch()

  if (gps_msgs::msg::GPSStatus::STATUS_NO_FIX != msg_.status.status) {
    msg_.status.satellites_used = msg.n_sats;
    msg_.status.position_source = 0;

    if (SBP_POS_LLH_FIX_MODE_DEAD_RECKONING !=
        SBP_POS_LLH_FIX_MODE_GET(msg.flags)) {
      msg_.status.position_source |= gps_msgs::msg::GPSStatus::SOURCE_GPS;
    }

    if (SBP_POS_LLH_INERTIAL_NAVIGATION_MODE_NONE !=
        SBP_POS_LLH_INERTIAL_NAVIGATION_MODE_GET(msg.flags)) {
      msg_.status.position_source |= gps_msgs::msg::GPSStatus::SOURCE_GYRO |
                                     gps_msgs::msg::GPSStatus::SOURCE_ACCEL;
    }

    msg_.latitude = msg.lat;     // [deg]
    msg_.longitude = msg.lon;    // [deg]
    msg_.altitude = msg.height;  // [m]

    msg_.position_covariance[0] = msg.cov_e_e;   // [m]
    msg_.position_covariance[1] = msg.cov_n_e;   // [m]
    msg_.position_covariance[2] = -msg.cov_e_d;  // [m]
    msg_.position_covariance[3] = msg.cov_n_e;   // [m]
    msg_.position_covariance[4] = msg.cov_n_n;   // [m]
    msg_.position_covariance[5] = -msg.cov_n_d;  // [m]
    msg_.position_covariance[6] = -msg.cov_e_d;  // [m]
    msg_.position_covariance[7] = -msg.cov_n_d;  // [m]
    msg_.position_covariance[8] = msg.cov_d_d;   // [m]
    msg_.position_covariance_type =
        gps_msgs::msg::GPSFix::COVARIANCE_TYPE_KNOWN;

    msg_.err_horz = Covariance::covarianceToEstimatedHorizonatalError(msg.cov_n_n, msg.cov_n_e, msg.cov_e_e) *
                    2.6926;                   // [m], scaled to 95% confidence
    msg_.err_vert = sqrt(msg.cov_d_d) * 2.0;  // [m], scaled to 95% confidence
    msg_.err = sqrt( msg_.err_horz*msg_.err_horz + msg_.err_vert*msg_.err_vert ); // [m], 95% confidence
  }
  else {
    msg_.position_covariance[0] = -1.0; // Position is invalid
  }

  last_received_pos_llh_cov_tow = msg.tow;

  publish();
}

void GPSFixPublisher::handle_sbp_msg(uint16_t sender_id,
                                     const sbp_msg_vel_ned_cov_t& msg) {
  if (0 == sender_id) return; // Ignore base station data

  if (SBP_VEL_NED_VELOCITY_MODE_INVALID !=
      SBP_VEL_NED_VELOCITY_MODE_GET(msg.flags)) {
    msg_.status.motion_source = 0;

    if (SBP_VEL_NED_VELOCITY_MODE_DEAD_RECKONING !=
        SBP_VEL_NED_VELOCITY_MODE_GET(msg.flags)) {
      msg_.status.motion_source |= gps_msgs::msg::GPSStatus::SOURCE_GPS;
    }

    if (SBP_VEL_NED_INS_NAVIGATION_MODE_NONE !=
        SBP_VEL_NED_INS_NAVIGATION_MODE_GET(msg.flags)) {
      msg_.status.motion_source |= gps_msgs::msg::GPSStatus::SOURCE_GYRO |
                                   gps_msgs::msg::GPSStatus::SOURCE_ACCEL;
    }

    msg_.speed = sqrt((double)(msg.n) * (double)(msg.n) +
                      (double)(msg.e) * (double)(msg.e)) /
                 1e3;                   // [m/s], horizontal
    msg_.climb = (double)msg.d / -1e3;  // [m/s], vertical

    msg_.err_speed =
        Covariance::covarianceToEstimatedHorizonatalError(msg.cov_n_n, msg.cov_n_e, msg.cov_e_e) *
        2.6926;                   // [m/s], scaled to 95% confidence
    msg_.err_climb =
        sqrt(msg.cov_d_d) * 2.0;  // [m/s], scaled to 95% confidence

    if (msg_.speed >= config_->getTrackUpdateMinSpeedMps()) {
      vel_ned_track_deg = Conversions::radiansToDegrees(atan2((double)msg.e, (double)msg.n));
      if (vel_ned_track_deg < 0.0) {
        vel_ned_track_deg += 360.0;
      }
      vel_ned_err_track_deg = Covariance::covarianceToEstimatedHorizonatalDirectionError(
                              (double)msg.n/1e3, (double)msg.e/1e3, msg.cov_n_n, msg.cov_e_e ) *
                              2.6926;  // [deg], scaled to 95% confidence
      vel_ned_track_valid = true;
    }
  }

  last_received_vel_ned_cov_tow = msg.tow;

  publish();
}

void GPSFixPublisher::handle_sbp_msg(uint16_t sender_id,
                                     const sbp_msg_orient_euler_t& msg) {
  if (0 == sender_id) return; // Ignore base station data

  if (SBP_ORIENT_EULER_INS_NAVIGATION_MODE_INVALID !=
      SBP_ORIENT_EULER_INS_NAVIGATION_MODE_GET(msg.flags)) {
    msg_.pitch = (double)msg.pitch / 1e6;  // [deg]
    msg_.roll = (double)msg.roll / 1e6;    // [deg]

    msg_.err_pitch =
        (double)msg.pitch_accuracy * 2.0;  // [deg], scaled to 95% confidence
    msg_.err_roll =
        (double)msg.roll_accuracy * 2.0;  // [deg], scaled to 95% confidence

    orientation_track_deg =
        (msg.yaw < 0) ? (double)msg.yaw / 1e6 + 360.0
                      : (double)msg.yaw / 1e6;  // [deg], in 0 to 360 range
    orientation_err_track_deg =
        (double)msg.yaw_accuracy * 2.0;  // [deg], scaled to 95% confidence
    orientation_track_valid = true;

    orientation_present = true;
  }

  last_received_orient_euler_tow = msg.tow;

  publish();
}

void GPSFixPublisher::handle_sbp_msg(uint16_t sender_id,
                                     const sbp_msg_dops_t& msg) {
  if (0 == sender_id) return; // Ignore base station data

  if (SBP_DOPS_FIX_MODE_INVALID != SBP_DOPS_FIX_MODE_GET(msg.flags)) {
    gdop = (double)msg.gdop / 1e2;
    pdop = (double)msg.pdop / 1e2;
    hdop = (double)msg.hdop / 1e2;
    vdop = (double)msg.vdop / 1e2;
    tdop = (double)msg.tdop / 1e2;

    time(&dops_time_s);
  }
}

void GPSFixPublisher::publish() {
  if (((last_received_gps_time_tow == last_received_utc_time_tow) ||
       !config_->getTimeStampSourceGNSS()) &&
      (last_received_gps_time_tow == last_received_pos_llh_cov_tow) &&
      (last_received_gps_time_tow == last_received_vel_ned_cov_tow) &&
      ((last_received_gps_time_tow == last_received_orient_euler_tow) ||
       !orientation_present)) {
    if (0 == msg_.header.stamp.sec) {
      // Use current platform time if time from the GNSS receiver is not
      // available or if a local time source is selected
      msg_.header.stamp = node_->now();
    }

    msg_.header.frame_id = frame_;

    if (orientation_track_valid) {
      // Use yaw for track if ORIENT EULER message is present and INS solution
      // is valid
      msg_.track = orientation_track_deg;
      msg_.err_track = orientation_err_track_deg;

      last_track_deg = msg_.track;
      last_err_track_deg = msg_.err_track;
      last_track_valid = true;

      orientation_track_valid = false;

      msg_.status.orientation_source =
          msg_.status.position_source;  // Orientation is provided by the INS
                                        // fusion only.
    } else if (vel_ned_track_valid) {
      // Use computed Course Over Ground (COG) for track if VEL NED COV message
      // is present and speed is valid
      msg_.track = vel_ned_track_deg;
      msg_.err_track = vel_ned_err_track_deg;

      last_track_deg = msg_.track;
      last_err_track_deg = msg_.err_track;
      last_track_valid = true;

      vel_ned_track_valid = false;
    } else if (last_track_valid) {
      // Use last valid track when there is no valid update
      msg_.track = last_track_deg;
      msg_.err_track = last_err_track_deg;
    }

    time_t current_time_s;
    time(&current_time_s);

    if (difftime(current_time_s, dops_time_s) <
        2.0) {  // Publish DOPs if not older than 2 seconds.
      msg_.gdop = gdop;
      msg_.pdop = pdop;
      msg_.hdop = hdop;
      msg_.vdop = vdop;
      msg_.tdop = tdop;
    }

    publisher_->publish(msg_);

    msg_ = gps_msgs::msg::GPSFix();
    last_received_gps_time_tow = -1;
    last_received_utc_time_tow = -2;
    last_received_pos_llh_cov_tow = -3;
    last_received_vel_ned_cov_tow = -4;
    last_received_orient_euler_tow = -5;
  }
}
