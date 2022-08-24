#include <publishers/NavSatFixPublisher.h>
#include <iostream>

#include <bitset>

constexpr uint32_t STATUS_MASK = 0x00000111;
constexpr uint32_t MAX_TIME_DIFF = 2000;

NavSatFixPublisher::NavSatFixPublisher(sbp::State* state,
                                       const std::string& topic_name,
                                       rclcpp::Node* node, const bool enabled, 
                                       const std::string& frame)
    : SBP2ROS2Publisher<sensor_msgs::msg::NavSatFix,
                        sbp_msg_obs_t, sbp_msg_pos_llh_cov_t>(
          state, topic_name, node, enabled, frame) {
            sbp_msg_obs_.header.t.tow = 0;
          }

void NavSatFixPublisher::handle_sbp_msg(
    uint16_t sender_id, const sbp_msg_obs_t& msg) {
  (void)sender_id;

  if (msg.header.t.tow != sbp_msg_obs_.header.t.tow){
    msg_ = sensor_msgs::msg::NavSatFix();
  }

  /**
   * As per sbp lib documentation:
   * 
   * 0 GPS L1CA
   * 1 GPS L2CM
   * 2 SBAS L1CA
   * 3 GLO L1CA
   * 4 GLO L2CA
   * 5 GPS L1P
   * 6 GPS L2P
   * 12 BDS2 B1
   * 13 BDS2 B2
   * 14 GAL E1B
   * 20 GAL E7I
   * 47 BDS3 B2a
   * 
   * NavStatus message definition:
   * SERVICE_GPS=1
   * SERVICE_GLONASS=2
   * SERVICE_COMPASS=4
   * SERVICE_GALILEO=8
   * 
   */
  // for (sbp_measurement_state_t state: msg.states) {
  //   if (state.mesid.code == 0 || 
  //       state.mesid.code == 1 ||
  //       state.mesid.code == 2 ||
  //       state.mesid.code == 5 ||
  //       state.mesid.code == 6 ) {
  //     msg_.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  //   } else if (state.mesid.code == 3 ||
  //              state.mesid.code == 4 ) {
  //     msg_.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
  //   } else if (state.mesid.code == 14 ||
  //              state.mesid.code == 20 ) {
  //     msg_.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
  //   } else if (state.mesid.code == 12 ||
  //              state.mesid.code == 13 ||
  //              state.mesid.code == 47 ) {
  //     msg_.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
  //   }
  // }

  sbp_msg_obs_ = msg;
  publish();
}

void NavSatFixPublisher::handle_sbp_msg(uint16_t sender_id,
                                        const sbp_msg_pos_llh_cov_t& msg) {
  (void)sender_id;
  
  // OBS msg has not arrived yet.
  if (sbp_msg_obs_.header.t.tow == 0) {
    return;
  }

  // Last received OBS msg tow is too old
  if ( (sbp_msg_obs_.header.t.tow - msg.tow) > MAX_TIME_DIFF ) {
    return;
  }

  loadCovarianceMatrix(msg);
  loadStatusFlag(msg);

  msg_.latitude = msg.lat;
  msg_.longitude = msg.lon;
  msg_.altitude = msg.height;

  publish();
}

void NavSatFixPublisher::loadCovarianceMatrix(const sbp_msg_pos_llh_cov_t& msg) {
  msg_.position_covariance[0] = msg.cov_e_e;
  msg_.position_covariance[1] = msg.cov_n_e;
  msg_.position_covariance[2] = -msg.cov_e_d;
  msg_.position_covariance[3] = msg.cov_n_e;
  msg_.position_covariance[4] = msg.cov_n_n;
  msg_.position_covariance[5] = -msg.cov_n_d;
  msg_.position_covariance[6] = -msg.cov_e_d;
  msg_.position_covariance[7] = -msg.cov_n_d;
  msg_.position_covariance[8] = msg.cov_d_d;

  msg_.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
}

void NavSatFixPublisher::loadStatusFlag(const sbp_msg_pos_llh_cov_t& msg) {

  // STATUS_NO_FIX=-1
  // STATUS_FIX=0
  // STATUS_SBAS_FIX=1 Satellite based augmentation
  // STATUS_GBAS_FIX=2 Ground based augmentation
  uint8_t status = msg.flags & STATUS_MASK;
  if(status == 0) {
    msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  } else if (status >= 2 && status <= 5) {
    msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
  } else if (status == 6) {
    msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
  }
}


void NavSatFixPublisher::publish() {
  if (enabled_) {

    msg_.header.stamp = node_->now();
    msg_.header.frame_id = frame_; 

    publisher_->publish(msg_);
  }
}