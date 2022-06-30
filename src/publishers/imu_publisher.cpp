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

#include <publishers/imu_publisher.h>
#include <utils/utils.h>
#include <climits>

ImuPublisher::ImuPublisher(sbp::State* state, const std::string_view topic_name,
                           rclcpp::Node* node, const LoggerPtr& logger,
                           const std::string& frame,
                           const std::shared_ptr<Config>& config)
    : SBP2ROS2Publisher<sensor_msgs::msg::Imu, sbp_msg_utc_time_t,
                        sbp_msg_gps_time_t, sbp_msg_gnss_time_offset_t,
                        sbp_msg_imu_aux_t, sbp_msg_imu_raw_t>(
          state, topic_name, node, logger, frame, config) {}

void ImuPublisher::handle_sbp_msg(uint16_t sender_id,
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

      linux_stamp_s_ = static_cast<double>(TimeUtils::utcToLinuxTime(utc)) +
                       static_cast<double>(msg.ns) / 1e9;
      last_received_utc_time_tow_ = msg.tow;
      compute_utc_offset();
    }
  }
}

void ImuPublisher::handle_sbp_msg(uint16_t sender_id,
                                  const sbp_msg_gps_time_t& msg) {
  if (0 == sender_id) return; // Ignore base station data

  if (config_->getTimeStampSourceGNSS()) {
    if (SBP_GPS_TIME_TIME_SOURCE_NONE !=
        SBP_GPS_TIME_TIME_SOURCE_GET(msg.flags)) {
      gps_week_ = msg.wn;
      gps_week_valid_ = true;

      gps_stamp_s_ = static_cast<double>(msg.wn * 604800u) +
                     static_cast<double>(msg.tow) / 1e3 +
                     static_cast<double>(msg.ns_residual) / 1e9;
      last_received_gps_time_tow_ = msg.tow;
      compute_utc_offset();
    }
  }
}

void ImuPublisher::compute_utc_offset( void ) {
  if (last_received_gps_time_tow_ == last_received_utc_time_tow_) {
    utc_offset_s_ = linux_stamp_s_ - gps_stamp_s_;
    utc_offset_valid_ = true;

    last_received_utc_time_tow_ = -1;
    last_received_gps_time_tow_ = -2;
  }
}


void ImuPublisher::handle_sbp_msg(uint16_t sender_id,
                                  const sbp_msg_gnss_time_offset_t& msg) {
  if (0 == sender_id) return; // Ignore base station data

  gps_time_offset_s_ = static_cast<double>(msg.weeks) * 604800.0 +
                       static_cast<double>(msg.milliseconds) / 1e3 +
                       static_cast<double>(msg.microseconds) / 1e6;
  gps_time_offset_valid_ = true;
}


void ImuPublisher::handle_sbp_msg(uint16_t sender_id,
                                  const sbp_msg_imu_aux_t& msg) {
  const double list_acc_res_mps2[] = {Conversions::standardGravityToMPS2(2.0) / 32768.0,
                                      Conversions::standardGravityToMPS2(4.0) / 32768.0,
                                      Conversions::standardGravityToMPS2(8.0) / 32768.0,
                                      Conversions::standardGravityToMPS2(16.0) / 32768.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0};

  const double list_gyro_res_rad[] = {Conversions::degreesToRadians(2000.0) / 32768.0,
                                      Conversions::degreesToRadians(1000.0) / 32768.0,
                                      Conversions::degreesToRadians(500.0) / 32768.0,
                                      Conversions::degreesToRadians(250.0) / 32768.0,
                                      Conversions::degreesToRadians(125.0) / 32768.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0};

  if (0 == sender_id) return; // Ignore base station data

  acc_res_mps2_ =
      list_acc_res_mps2[SBP_IMU_AUX_ACCELEROMETER_RANGE_GET(msg.imu_conf)];

  gyro_res_rad_ =
      list_gyro_res_rad[SBP_IMU_AUX_GYROSCOPE_RANGE_GET(msg.imu_conf)];
}


void ImuPublisher::handle_sbp_msg(uint16_t sender_id,
                                  const sbp_msg_imu_raw_t& msg) {
  uint32_t imu_raw_tow_ms;
  double timestamp_s = 0.0;

  if (0 == sender_id) return; // Ignore base station data

  if ( config_->getTimeStampSourceGNSS() ) {

    switch ( SBP_IMU_RAW_TIME_STATUS_GET(msg.tow) ) {
      case SBP_IMU_RAW_TIME_STATUS_REFERENCE_EPOCH_IS_START_OF_CURRENT_GPS_WEEK:
        if (gps_week_valid_ && utc_offset_valid_) {
          imu_raw_tow_ms = SBP_IMU_RAW_TIME_SINCE_REFERENCE_EPOCH_IN_MILLISECONDS_GET(msg.tow);

          // Check for TOW rollover before the next GPS TIME message arrives
          if ( (gps_week_ == last_gps_week_) && (imu_raw_tow_ms < last_imu_raw_tow_ms_) ) {
            gps_week_++;
          }
          last_gps_week_ = gps_week_;
          last_imu_raw_tow_ms_ = imu_raw_tow_ms;

          timestamp_s =
              static_cast<double>(gps_week_ * 604800u) +
              static_cast<double>(imu_raw_tow_ms) / 1e3 +
              static_cast<double>(msg.tow_f) / 1e3 / 256.0 + utc_offset_s_;
          stamp_source_ = STAMP_SOURCE_GNSS;
        }
        break;

      case SBP_IMU_RAW_TIME_STATUS_REFERENCE_EPOCH_IS_TIME_OF_SYSTEM_STARTUP:
        if (gps_time_offset_valid_ && utc_offset_valid_) {
          imu_raw_tow_ms = SBP_IMU_RAW_TIME_SINCE_REFERENCE_EPOCH_IN_MILLISECONDS_GET(msg.tow);
          timestamp_s =
              gps_time_offset_s_ +
              static_cast<double>(imu_raw_tow_ms) / 1e3 +
              static_cast<double>(msg.tow_f) / 1e3 / 256.0 + utc_offset_s_;
          stamp_source_ = STAMP_SOURCE_GNSS;
        }
        break;

    } // switch()

    msg_.header.stamp.sec     = static_cast<uint32_t>(timestamp_s);
    msg_.header.stamp.nanosec = static_cast<uint32_t>( (timestamp_s - static_cast<double>(msg_.header.stamp.sec)) * 1e9 );
  }

  msg_.orientation_covariance[0] = -1.0;  // Orientation is not provided

  if ((acc_res_mps2_ - 0.0) > std::numeric_limits<double>::epsilon()) {
    msg_.linear_acceleration.x = static_cast<double>(msg.acc_x) * acc_res_mps2_;
    msg_.linear_acceleration.y = static_cast<double>(msg.acc_y) * acc_res_mps2_;
    msg_.linear_acceleration.z = static_cast<double>(msg.acc_z) * acc_res_mps2_;
  } else {
    msg_.linear_acceleration_covariance[0] = -1.0;  // Acceleration is not valid
  }

  if ((gyro_res_rad_ - 0.0) > std::numeric_limits<double>::epsilon()) {
    msg_.angular_velocity.x = static_cast<double>(msg.gyr_x) * gyro_res_rad_;
    msg_.angular_velocity.y = static_cast<double>(msg.gyr_y) * gyro_res_rad_;
    msg_.angular_velocity.z = static_cast<double>(msg.gyr_z) * gyro_res_rad_;
  } else {
    msg_.angular_velocity_covariance[0] = -1.0; // Angular velocity is not valid
  }

  publish();
}


void ImuPublisher::publish() {
  if ( 0 == msg_.header.stamp.sec ) {
    // Use current platform time if time from the GNSS receiver is not
    // available or if a local time source is selected
    msg_.header.stamp = node_->now();
    stamp_source_ = STAMP_SOURCE_PLATFORM;
  }

  if ( stamp_source_ != last_stamp_source_ ) {
    // Time stamp source has changed - invalidate measurements
    msg_.linear_acceleration_covariance[0] = -1.0;
    msg_.angular_velocity_covariance[0] = -1.0;
  }
  last_stamp_source_ = stamp_source_;

  msg_.header.frame_id = frame_;

  publisher_->publish(msg_);

  msg_ = sensor_msgs::msg::Imu();
}
