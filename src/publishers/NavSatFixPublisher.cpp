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

  std::cout << "msg.header.t.tow: " << msg.header.t.tow  << std::endl;
  std::cout << "sbp_msg_obs_.header.t.tow: " << sbp_msg_obs_.header.t.tow  << std::endl;

  if (msg.header.t.tow != sbp_msg_obs_.header.t.tow){
    msg_ = sensor_msgs::msg::NavSatFix();
  }

  /**
   * As per sbp lib documentation:
   * 
   * 0  GPS L1CA
   * 1  GPS L2CM
   * 5  GPS L1P
   * 6  GPS L2P
   * 7  GPS L2CL
   * 8  GPS L2CX
   * 9  GPS L5I
   * 10 GPS L5Q
   * 11 GPS L5X
   * 56 GPS L1CI
   * 57 GPS L1CQ
   * 58 GPS L1CX
   * 
   * 2  SBAS L1CA
   * 41 SBAS L5I
   * 42 SBAS L5Q
   * 43 SBAS L5X
   * 
   * 3  GLO L1OF
   * 4  GLO L20F
   * 29 GLO L1P
   * 30 GLO L2P
   * 
   * 12 BDS2 B1
   * 13 BDS2 B2
   * 44 BDS3 B1CI
   * 45 BDS3 B1CQ
   * 46 BDS3 B1CX
   * 47 BDS3 B5I
   * 48 BDS3 B5Q
   * 49 BDS3 B5X
   * 50 BDS3 B7I
   * 51 BDS3 B7Q
   * 52 BDS3 B7X
   * 53 BDS3 B3I
   * 54 BDS3 B3Q
   * 55 BDS3 B3X
   * 
   * 14 GAL E1B
   * 15 GAL E1C
   * 16 GAL E1X
   * 17 GAL E6B
   * 18 GAL E6C
   * 19 GAL E6X
   * 20 GAL E7I
   * 21 GAL E7Q
   * 22 GAL E7X
   * 23 GAL E8I
   * 24 GAL E8I
   * 25 GAL E8X
   * 26 GAL E5I
   * 27 GAL E5Q
   * 28 GAL E5X
   *
   * 31 QZS L1CA
   * 32 QZS L1CI
   * 33 QZS L1CQ
   * 34 QZS L1CX
   * 35 QZS L2CM
   * 36 QZS L2CL
   * 
   * NavStatus message definition:
   * SERVICE_GPS=1
   * SERVICE_GLONASS=2
   * SERVICE_COMPASS=4
   * SERVICE_GALILEO=8
   * 
   */
  sbp_packed_obs_content_t obs_content;
  for(int i = 0; i < msg.n_obs; i++) {
    obs_content = msg.obs[i];

    if (obs_content.sid.code == 0  ||  
        obs_content.sid.code == 1  ||
        obs_content.sid.code == 5  ||
        obs_content.sid.code == 7  ||
        obs_content.sid.code == 8  ||
        obs_content.sid.code == 9  ||
        obs_content.sid.code == 10 ||
        obs_content.sid.code == 11 ||
        obs_content.sid.code == 56 ||
        obs_content.sid.code == 57 ||
        obs_content.sid.code == 58 ) {
        msg_.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
      } else if (obs_content.sid.code == 2  || 
                obs_content.sid.code == 41 ||  
                obs_content.sid.code == 42 ||  
                obs_content.sid.code == 43 ) {
                  msg_.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
      } else if (obs_content.sid.code >= 31  &&  obs_content.sid.code <= 40 ) {
                  msg_.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
      } else if (obs_content.sid.code == 3  ||
                obs_content.sid.code == 4  ||
                obs_content.sid.code == 29 ||
                obs_content.sid.code == 30 ) {
        msg_.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
      } else if (obs_content.sid.code >= 14 &&
                obs_content.sid.code <= 28 ) {
        msg_.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
      } else if (obs_content.sid.code == 12 ||
                obs_content.sid.code == 13 || 
                ( obs_content.sid.code >= 44 && obs_content.sid.code <= 55 )) {
        msg_.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
      }
  }

  sbp_msg_obs_ = msg;  
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
  if(status == 0 || status == 5) {
    msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  } else if (status == 1) {
    msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  } else if (status >= 2 && status <= 4) {
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