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

#include <publishers/navsatfix_publisher.h>
#include <utils/utils.h>

// GNSS Signal Code Identifier
typedef enum {
  CODE_GPS_L1CA = 0,
  CODE_GPS_L2CM = 1,
  CODE_SBAS_L1CA = 2,
  CODE_GLO_L1OF = 3,
  CODE_GLO_L2OF = 4,
  CODE_GPS_L1P = 5,
  CODE_GPS_L2P = 6,
  CODE_GPS_L2CL = 7,
  CODE_GPS_L2CX = 8,
  CODE_GPS_L5I = 9,
  CODE_GPS_L5Q = 10,
  CODE_GPS_L5X = 11,
  CODE_BDS2_B1 = 12,
  CODE_BDS2_B2 = 13,
  CODE_GAL_E1B = 14,
  CODE_GAL_E1C = 15,
  CODE_GAL_E1X = 16,
  CODE_GAL_E6B = 17,
  CODE_GAL_E6C = 18,
  CODE_GAL_E6X = 19,
  CODE_GAL_E7I = 20,
  CODE_GAL_E7Q = 21,
  CODE_GAL_E7X = 22,
  CODE_GAL_E8I = 23,
  CODE_GAL_E8Q = 24,
  CODE_GAL_E8X = 25,
  CODE_GAL_E5I = 26,
  CODE_GAL_E5Q = 27,
  CODE_GAL_E5X = 28,
  CODE_GLO_L1P = 29,
  CODE_GLO_L2P = 30,
  CODE_BDS3_B1CI = 44,
  CODE_BDS3_B1CQ = 45,
  CODE_BDS3_B1CX = 46,
  CODE_BDS3_B5I = 47,
  CODE_BDS3_B5Q = 48,
  CODE_BDS3_B5X = 49,
  CODE_BDS3_B7I = 50,
  CODE_BDS3_B7Q = 51,
  CODE_BDS3_B7X = 52,
  CODE_BDS3_B3I = 53,
  CODE_BDS3_B3Q = 54,
  CODE_BDS3_B3X = 55,
  CODE_GPS_L1CI = 56,
  CODE_GPS_L1CQ = 57,
  CODE_GPS_L1CX = 58
} gnss_signal_code_t;

NavSatFixPublisher::NavSatFixPublisher(sbp::State* state,
                                       const std::string_view topic_name,
                                       rclcpp::Node* node,
                                       const LoggerPtr& logger,
                                       const std::string& frame,
                                       const std::shared_ptr<Config>& config)
    : SBP2ROS2Publisher<sensor_msgs::msg::NavSatFix,
                        sbp_msg_measurement_state_t, sbp_msg_utc_time_t,
                        sbp_msg_pos_llh_cov_t>(state, topic_name, node, logger,
                                               frame, config) {}

void NavSatFixPublisher::handle_sbp_msg(
    uint16_t sender_id, const sbp_msg_measurement_state_t& msg) {
  sbp_measurement_state_t state;

  if (0 == sender_id) return; // Ignore base station data

  status_service = 0;

  for (int i = 0; i < msg.n_states; i++) {
    state = msg.states[i];

    if (state.cn0 > 0) {
      switch (state.mesid.code) {
        case CODE_GPS_L1CA:
        case CODE_GPS_L2CM:
        case CODE_GPS_L1P:
        case CODE_GPS_L2P:
        case CODE_GPS_L2CL:
        case CODE_GPS_L2CX:
        case CODE_GPS_L5I:
        case CODE_GPS_L5Q:
        case CODE_GPS_L5X:
        case CODE_GPS_L1CI:
        case CODE_GPS_L1CQ:
        case CODE_GPS_L1CX:
          status_service |= sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
          break;

        case CODE_GLO_L1OF:
        case CODE_GLO_L2OF:
        case CODE_GLO_L1P:
        case CODE_GLO_L2P:
          status_service |= sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
          break;

        case CODE_GAL_E1B:
        case CODE_GAL_E1C:
        case CODE_GAL_E1X:
        case CODE_GAL_E6B:
        case CODE_GAL_E6C:
        case CODE_GAL_E6X:
        case CODE_GAL_E7I:
        case CODE_GAL_E7Q:
        case CODE_GAL_E7X:
        case CODE_GAL_E8I:
        case CODE_GAL_E8Q:
        case CODE_GAL_E8X:
        case CODE_GAL_E5I:
        case CODE_GAL_E5Q:
        case CODE_GAL_E5X:
          status_service |= sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
          break;

        case CODE_BDS2_B1:
        case CODE_BDS2_B2:
        case CODE_BDS3_B1CI:
        case CODE_BDS3_B1CQ:
        case CODE_BDS3_B1CX:
        case CODE_BDS3_B5I:
        case CODE_BDS3_B5Q:
        case CODE_BDS3_B5X:
        case CODE_BDS3_B7I:
        case CODE_BDS3_B7Q:
        case CODE_BDS3_B7X:
        case CODE_BDS3_B3I:
        case CODE_BDS3_B3Q:
        case CODE_BDS3_B3X:
          status_service |= sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
          break;

      }  // switch()
    }    // if()
  }      // for()
}

void NavSatFixPublisher::handle_sbp_msg(uint16_t sender_id,
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

    last_received_utc_time_tow = msg.tow;

    publish();
  }
}

void NavSatFixPublisher::handle_sbp_msg(uint16_t sender_id,
                                        const sbp_msg_pos_llh_cov_t& msg) {
  if (0 == sender_id) return; // Ignore base station data

  switch (SBP_POS_LLH_FIX_MODE_GET(msg.flags)) {
    case SBP_POS_LLH_FIX_MODE_SINGLE_POINT_POSITION:
      msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      break;
    case SBP_POS_LLH_FIX_MODE_DIFFERENTIAL_GNSS:
      msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      break;
    case SBP_POS_LLH_FIX_MODE_FLOAT_RTK:
      msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      break;
    case SBP_POS_LLH_FIX_MODE_FIXED_RTK:
      msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      break;
    case SBP_POS_LLH_FIX_MODE_DEAD_RECKONING:
      msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      break;
    case SBP_POS_LLH_FIX_MODE_SBAS_POSITION:
      msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      break;
    default:
      msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  }  // switch()

  if (sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX != msg_.status.status) {
    msg_.status.service = status_service;

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
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
  }
  else {
    msg_.position_covariance[0] = -1.0; // Position is invalid
  }

  last_received_pos_llh_cov_tow = msg.tow;

  publish();
}

void NavSatFixPublisher::publish() {
  if ((last_received_pos_llh_cov_tow == last_received_utc_time_tow) ||
      !config_->getTimeStampSourceGNSS()) {
    if (0 == msg_.header.stamp.sec) {
      // Use current platform time if time from the GNSS receiver is not
      // available or if a local time source is selected
      msg_.header.stamp = node_->now();
    }

    msg_.header.frame_id = frame_;

    publisher_->publish(msg_);

    msg_ = sensor_msgs::msg::NavSatFix();
    last_received_utc_time_tow = -1;
    last_received_pos_llh_cov_tow = -2;
  }
}
