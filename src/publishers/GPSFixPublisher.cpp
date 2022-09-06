#include<publishers/GPSFixPublisher.h>

GPSFixPublisher::GPSFixPublisher(sbp::State* state, const std::string& topic_name,
                     rclcpp::Node* node, const LoggerPtr& logger,
                     const bool enabled, const std::string& frame): SBP2ROS2Publisher<
                                                    gps_msgs::msg::GPSFix,
                                                    sbp_msg_pos_llh_acc_t,
                                                    sbp_msg_pos_llh_cov_t,
                                                    sbp_msg_vel_cog_t,
                                                    sbp_msg_vel_ned_cov_t,
                                                    sbp_msg_orient_euler_t,
                                                    sbp_msg_dops_t,
                                                    sbp_msg_gps_time_t>(
                                state, topic_name, node, logger, enabled, frame) {}

void GPSFixPublisher::handle_sbp_msg(uint16_t sender_id, const sbp_msg_pos_llh_acc_t& msg){
    msg_.status.satellites_used = msg.n_sats;

    msg_.err_horz = msg.h_accuracy;
    msg_.err_vert = msg.v_accuracy;
    msg_.err_track = msg.at_accuracy;
                        
}

void GPSFixPublisher::handle_sbp_msg(uint16_t sender_id, const sbp_msg_pos_llh_cov_t& msg){
    msg_.latitude = msg.lat;
    msg_.longitude = msg.lon;
    msg_.altitude = msg.height;
    msg_.time = msg.tow;

    loadCovarianceMatrix(msg);
}

void GPSFixPublisher::handle_sbp_msg(uint16_t sender_id, const sbp_msg_vel_cog_t& msg){
    msg_.track = msg.cog;
    msg_.speed = msg.sog;
    msg_.climb = msg.v_up;
    msg_.err_speed = msg.sog_accuracy;
    msg_.err_climb = msg.v_up_accuracy;
}

void GPSFixPublisher::handle_sbp_msg(uint16_t sender_id, const sbp_msg_vel_ned_cov_t& msg){
    //TODO what to do with this msg?
}

void GPSFixPublisher::handle_sbp_msg(uint16_t sender_id, const sbp_msg_orient_euler_t& msg){
    msg_.pitch = msg.pitch;
    msg_.roll = msg.roll;
    msg_.dip = msg.yaw;
    msg_.err_pitch = msg.pitch_accuracy;
    msg_.err_roll = msg.roll_accuracy;
    msg_.err_dip = msg.yaw_accuracy;
}

void GPSFixPublisher::handle_sbp_msg(uint16_t sender_id, const sbp_msg_dops_t& msg){
    msg_.gdop = msg.gdop;
    msg_.pdop = msg.pdop;
    msg_.hdop = msg.hdop;
    msg_.vdop = msg.vdop;
    msg_.tdop = msg.tdop;                        
}

void GPSFixPublisher::handle_sbp_msg(uint16_t sender_id, const sbp_msg_gps_time_t& msg){
    //TODO what to do with this msg?
}

void GPSFixPublisher::publish(){

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

  msg_.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
}