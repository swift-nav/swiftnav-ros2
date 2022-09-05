#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <libsbp/cpp/state.h>
#include <libsbp/cpp/message_handler.h>

#include <publishers/SBP2ROS2Publisher.h>

/**
 * @brief Class that listens for sbp_msg_obs_t and sbp_msg_pos_llh_cov_t, 
 * publishing a sensor_msgs::msg::NavSatFix ros2 message.
 *   
*/
class NavSatFixPublisher : public SBP2ROS2Publisher<sensor_msgs::msg::NavSatFix,
                                                    sbp_msg_obs_t,
                                                    sbp_msg_pos_llh_cov_t> {
 public:
  NavSatFixPublisher() = delete;

  /**
   * @brief Construct a new Nav Sat Fix Publisher object
   * 
   * @param state SBP State object
   * @param topic_name Name of the topic to publish a sensor_msgs::msg::NavSatFix message
   * @param node ROS 2 node object
   * @param enabled Flag telling if the topic should be published (true) or not
   * (false)
   */
  NavSatFixPublisher(sbp::State* state, const std::string& topic_name,
                     rclcpp::Node* node, const LoggerPtr& logger,
                     const bool enabled, const std::string& frame);

  /**
   * @brief Handles a sbp_msg_measurement_state_t message. It gets the constellation for
   * each satellite in the measurement states.
   * 
   * As per sbp lib documentation, the code for satellite constelation are:
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
   * Ros2 NavStatus message definition for service:
   * SERVICE_GPS=1
   * SERVICE_GLONASS=2
   * SERVICE_COMPASS=4
   * SERVICE_GALILEO=8
   * 
   * Mapping:
   * 
   *  GPS L1CA, SBAS L1CA, GPS L2CM, GPS L1P, GPS L2P => SERVICE_GPS
   *  GLO L1CA, GLO L2CA => SERVICE_GLONASS
   *  GAL E1B, GAL E7I => SERVICE_GALILEO
   *  BDS2 B1, BDS2 B2, BDS3 B2a => SERVICE_COMPASS
   * 
   * @param sender_id Ignored
   * @param msg Incoming sbp_msg_obs_t 
   */
  void handle_sbp_msg(uint16_t sender_id,
                      const sbp_msg_obs_t& msg);

  /**
   * @brief Handles a sbp_msg_pos_llh_cov_t message. It gets the latitude, longitude
   * and altitude from the sbp message and calls methods for covariance matrix and 
   * status flag.
   * 
   * @param sender_id Ignored
   * @param msg Incoming sbp_msg_pos_llh_cov_t 
   */
  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_pos_llh_cov_t& msg);

 protected:
  /**
   * @brief Checks that the Ros2 sensor_msgs::msg::NavSatFix is complete, if so, 
   * it publishes it
   * 
   */
  void publish() override;

 private:
  
  /**
   * @brief Loads the covariance matrix values in the ROS2 message from values in the
   * sbp message. To do so it converts the covariance matrix from NED to ENU.
   * 
   * @param msg sbp_msg_pos_llh_cov_t
   */
  void loadCovarianceMatrix(const sbp_msg_pos_llh_cov_t& msg);
  
  /**
   * @brief Loads the status flag in the ROS2 message from values in the sbp_msg_pos_llh_cov_t 
   * message.
   * 
   * As per sbp library documentation:
   * 
   *  0 Invalid
   *  1 Single Point Position (SPP)
   *  2 Differential GNSS (DGNSS)
   *  3 Float RTK
   *  4 Fixed RTK
   *  5 Dead Reckoning
   *  6 SBAS Position
   * 
   * As per Ros2 sensor_msgs::msg::NavSatStatus message:
   *   STATUS_NO_FIX=-1
   *   STATUS_FIX=0
   *   STATUS_SBAS_FIX=1 Satellite based augmentation
   *   STATUS_GBAS_FIX=2 Ground based augmentation
   * 
   * Mapping:
   *   Invalid => STATUS_NO_FIX
   *   Single Point Position (SPP) => STATUS_FIX
   *   Differential GNSS (DGNSS), Float RTK, Fixed RTK, Dead Reckoning => STATUS_GBAS_FIX
   *   SBAS Position => STATUS_SBAS_FIX
   * 
   * @param msg sbp_msg_pos_llh_cov_t
   */
  void loadStatusFlag(const sbp_msg_pos_llh_cov_t& msg);

  sbp_msg_obs_t sbp_msg_obs_;
};