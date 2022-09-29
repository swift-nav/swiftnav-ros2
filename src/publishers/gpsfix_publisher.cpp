#include <publishers/gpsfix_publisher.h>

GPSFixPublisher::GPSFixPublisher(sbp::State* state,
                                 const std::string& topic_name,
                                 rclcpp::Node* node, const LoggerPtr& logger,
                                 const std::string& frame)
    : SBP2ROS2Publisher<gps_msgs::msg::GPSFix, sbp_msg_pos_llh_acc_t,
                        sbp_msg_pos_llh_cov_t, sbp_msg_vel_cog_t,
                        sbp_msg_vel_ned_cov_t, sbp_msg_orient_euler_t,
                        sbp_msg_dops_t, sbp_msg_gps_time_t, sbp_msg_obs_t>(
          state, topic_name, node, logger, frame) {}

void GPSFixPublisher::handle_sbp_msg(uint16_t sender_id,
                                     const sbp_msg_pos_llh_acc_t& msg) {
  (void)sender_id;
  msg_.status.satellites_used = msg.n_sats;
  // msg_.satellite_used_prn = ?
  msg_.err_horz = msg.h_accuracy;
  msg_.err_vert = msg.v_accuracy;
  msg_.err_track = msg.at_accuracy;

  if (ok_to_publish(msg.tow)) {
    publish();
  }

  return;
}

void GPSFixPublisher::handle_sbp_msg(uint16_t sender_id,
                                     const sbp_msg_pos_llh_cov_t& msg) {
  (void)sender_id;
  msg_.latitude = msg.lat;
  msg_.longitude = msg.lon;
  msg_.altitude = msg.height;
  msg_.time = msg.tow;

  loadCovarianceMatrix(msg);
  last_received_pos_llh_cov_tow_ = msg.tow;
}

void GPSFixPublisher::handle_sbp_msg(uint16_t sender_id,
                                     const sbp_msg_vel_cog_t& msg) {
  (void)sender_id;
  msg_.track = msg.cog;
  msg_.speed = msg.sog;
  msg_.climb = msg.v_up;
  msg_.err_speed = msg.sog_accuracy;
  msg_.err_climb = msg.v_up_accuracy;

  last_received_vel_cog_tow_ = msg.tow;
}

void GPSFixPublisher::handle_sbp_msg(uint16_t sender_id,
                                     const sbp_msg_vel_ned_cov_t& msg) {
  (void)sender_id;
  last_received_vel_ned_cov_tow_ = msg.tow;
  // TODO are we using this for something? else remove
}

void GPSFixPublisher::handle_sbp_msg(uint16_t sender_id,
                                     const sbp_msg_orient_euler_t& msg) {
  (void)sender_id;
  msg_.pitch = msg.pitch;
  msg_.roll = msg.roll;
  msg_.dip = msg.yaw;
  msg_.err_pitch = msg.pitch_accuracy;
  msg_.err_roll = msg.roll_accuracy;
  msg_.err_dip = msg.yaw_accuracy;

  last_received_orient_euler_tow_ = msg.tow;
}

void GPSFixPublisher::handle_sbp_msg(uint16_t sender_id,
                                     const sbp_msg_dops_t& msg) {
  (void)sender_id;
  msg_.gdop = msg.gdop;
  msg_.pdop = msg.pdop;
  msg_.hdop = msg.hdop;
  msg_.vdop = msg.vdop;
  msg_.tdop = msg.tdop;

  last_received_dops_tow_ = msg.tow;
}

void GPSFixPublisher::handle_sbp_msg(uint16_t sender_id,
                                     const sbp_msg_gps_time_t& msg) {
  (void)sender_id;
  last_received_gps_time_tow_ = msg.tow;
}

void GPSFixPublisher::handle_sbp_msg(uint16_t sender_id,
                                     const sbp_msg_obs_t& msg) {
  (void)sender_id;
  last_received_obs_tow_ = msg.header.t.tow;

  msg_.status.satellites_visible = msg.n_obs;
  sbp_packed_obs_content_t obs_content;
  for (int i = 0; i < msg.n_obs; i++) {
    obs_content = msg.obs[i];
    msg_.status.satellite_visible_prn.push_back(obs_content.sid.code);
  }
}

bool GPSFixPublisher::ok_to_publish(const u32& tow) const {
  const u32 pos_llh_cov_time_diff = (last_received_pos_llh_cov_tow_ > tow)
                                        ? last_received_pos_llh_cov_tow_ - tow
                                        : tow - last_received_pos_llh_cov_tow_;

  const u32 vel_cog_time_diff = (last_received_vel_cog_tow_ > tow)
                                    ? last_received_vel_cog_tow_ - tow
                                    : tow - last_received_vel_cog_tow_;

  const u32 vel_ned_cov_time_diff = (last_received_vel_ned_cov_tow_ > tow)
                                        ? last_received_vel_ned_cov_tow_ - tow
                                        : tow - last_received_vel_ned_cov_tow_;

  const u32 orient_euler_time_diff =
      (last_received_orient_euler_tow_ > tow)
          ? last_received_orient_euler_tow_ - tow
          : tow - last_received_orient_euler_tow_;

  const u32 dops_time_diff = (last_received_dops_tow_ > tow)
                                 ? last_received_dops_tow_ - tow
                                 : tow - last_received_dops_tow_;

  const u32 gps_time_time_diff = (last_received_gps_time_tow_ > tow)
                                     ? last_received_gps_time_tow_ - tow
                                     : tow - last_received_gps_time_tow_;

  const u32 obs_time_diff = (last_received_obs_tow_ > tow)
                                ? last_received_obs_tow_ - tow
                                : tow - last_received_obs_tow_;

  if (pos_llh_cov_time_diff > MAX_POS_LLH_COV_TIME_DIFF) {
    return false;
  } else if (vel_cog_time_diff > MAX_VEL_COG_TIME_DIFF) {
    if(last_received_vel_cog_tow_ != 0){
      return false;
    }
  } else if (vel_ned_cov_time_diff > MAX_VEL_NED_COV_TIME_DIFF) {
    if(last_received_vel_ned_cov_tow_ != 0){
      return false;
    }
  } else if (orient_euler_time_diff > MAX_ORIENT_EULER_TIME_DIFF) {
    if(last_received_orient_euler_tow_ != 0){
      return false;
    }
  } else if (dops_time_diff > MAX_DOPS_TIME_DIFF) {
    return false;
  } else if (gps_time_time_diff > MAX_GPS_TIME_TIME_DIFF) {
    return false;
  } else if (obs_time_diff > MAX_OBS_TIME_DIFF_MS) {
    return false;
  }
  return true;
}

void GPSFixPublisher::publish() {
  msg_.header.stamp = node_->now();
  msg_.header.frame_id = frame_;
  publisher_->publish(msg_);
}

void GPSFixPublisher::loadCovarianceMatrix(const sbp_msg_pos_llh_cov_t& msg) {
  msg_.position_covariance[0] = msg.cov_e_e;
  msg_.position_covariance[1] = msg.cov_n_e;
  msg_.position_covariance[2] = -msg.cov_e_d;
  msg_.position_covariance[3] = msg.cov_n_e;
  msg_.position_covariance[4] = msg.cov_n_n;
  msg_.position_covariance[5] = -msg.cov_n_d;
  msg_.position_covariance[6] = -msg.cov_e_d;
  msg_.position_covariance[7] = -msg.cov_n_d;
  msg_.position_covariance[8] = msg.cov_d_d;

  msg_.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_KNOWN;
}