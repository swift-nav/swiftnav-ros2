#pragma once

#include <rclcpp/rclcpp.hpp>
#include <gps_common/msg/gps_fix.hpp>

#include <libsbp/cpp/state.h>
#include <libsbp/cpp/message_handler.h>

#include <publishers/SBP2ROS2Publisher.h>

/**
 * @brief Class that listens for sbp_msg_obs_t and sbp_msg_pos_llh_cov_t, 
 * publishing a gps_comon::msg::GPSFix ros2 message.
 *   
*/
class GPSFixPublisher : public SBP2ROS2Publisher<gps_comon::msg::GPSFix,
                                                    sbp_msg_pos_llh_acc_t,
                                                    sbp_msg_pos_llh_cov_t,
                                                    sbp_msg_vel_cog_t,
                                                    sbp_msg_vel_ned_cov_t,
                                                    sbp_msg_orient_euler_t,
                                                    sbp_msg_dops_t,
                                                    sbp_msg_gps_time_t
                                                    > {
 public:
  GPSFixPublisher() = delete;

  /**
   * @brief Construct a new Gps Fix Publisher object
   * 
   * @param state SBP State object
   * @param topic_name Name of the topic to publish a gps_comon::msg::GPSFix message
   * @param node ROS 2 node object
   * @param enabled Flag telling if the topic should be published (true) or not
   * (false)
   */
  GPSFixPublisher(sbp::State* state, const std::string& topic_name,
                     rclcpp::Node* node, const LoggerPtr& logger,
                     const bool enabled, const std::string& frame);

  
  void handle_sbp_msg(uint16_t sender_id,
                      const sbp_msg_obs_t& msg);

  void handle_sbp_msg(uint16_t sender_id,
                      const sbp_msg_pos_llh_acc_t& msg);

  void handle_sbp_msg(uint16_t sender_id,
                      const sbp_msg_pos_llh_cov_t& msg);

  void handle_sbp_msg(uint16_t sender_id,
                      const sbp_msg_vel_cog_t& msg);

  void handle_sbp_msg(uint16_t sender_id,
                      const sbp_msg_vel_ned_cov_t& msg);

  void handle_sbp_msg(uint16_t sender_id,
                      const sbp_msg_orient_euler_t& msg);

  void handle_sbp_msg(uint16_t sender_id,
                      const sbp_msg_dops_t& msg);

  void handle_sbp_msg(uint16_t sender_id,
                      const sbp_msg_gps_time_t& msg);

  
 protected:
  /**
   * @brief Checks that the Ros2 gps_comon::msg::GPSFix is complete, if so, 
   * it publishes it
   * 
   */
  void publish() override;

 private:
    
  sbp_msg_obs_t sbp_msg_obs_;
  sbp_msg_pos_llh_acc_t sbp_msg_pos_llh_acc;
  sbp_msg_pos_llh_cov_t sbp_msg_pos_llh_cov;
  sbp_msg_vel_cog_t sbp_msg_vel_cog;
  sbp_msg_vel_ned_cov_t sbp_msg_vel_ned_cov;
  sbp_msg_orient_euler_t sbp_msg_orient_euler;
  sbp_msg_dops_t sbp_msg_dops;
  sbp_msg_gps_time_t sbp_msg_gps_time;
};