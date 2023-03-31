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

#pragma once

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

/**
 * @brief Class that manages the ROS node configuration
 */
class Config {
 public:
  Config() = delete;

  /**
   * @brief Construct a new Config object
   *
   * @param node ROS 2 node
   */
  explicit Config(rclcpp::Node* node);

  // Getters
  std::string getFrame() const { return frame_; }
  bool getLogSBPMessages() const { return log_sbp_messages_; }
  std::string getLogPath() const { return log_path_; }
  int32_t getInterface() const { return interface_; }
  std::string getFile() const { return file_; }
  std::string getDevice() const { return device_; }
  std::string getConnectionString() const { return connection_str_; }
  int32_t getReadTimeout() const { return read_timeout_; }
  int32_t getWriteTimeout() const { return write_timeout_; }
  std::string getIP() const { return ip_; }
  int32_t getPort() const { return port_; }
  std::vector<std::string> getPublishers() const { return enabled_publishers_; }
  bool getTimeStampSourceGNSS() const { return timestamp_source_gnss_; }
  double getBaseLineDirOffsetDeg() const { return baseline_dir_offset_deg_; }
  double getBaseLineDipOffsetDeg() const { return baseline_dip_offset_deg_; }
  double getTrackUpdateMinSpeedMps() const {
    return track_update_min_speed_mps_;
  }

 private:
  /**
   * @brief Declares the parameters the ROS node will use
   *
   * @param node ROS 2 node
   */
  void declareParameters(rclcpp::Node* node);

  /**
   * @brief Loads the declared parameters from the settings.yaml file
   *
   * @param node ROS 2 node
   */
  void loadParameters(rclcpp::Node* node);

  std::string frame_;
  bool log_sbp_messages_; /** @brief Flag to enable/disable SBP messages logging
                           */
  std::string
      log_path_;       /** @brief Complete path for SBP message logging file */
  int32_t interface_;  /** @brief ID of the Data Source type for SBP messages */
  std::string file_;   /** @brief Complete path to the file containing SBP
                          messages to use as input */
  std::string device_; /** @brief Serial device name used as input (in OS native
                          format e.g. /dev/ttyS0) */
  std::string connection_str_; /** @brief Connection string used to parametrize
                                  serial port */
  int32_t read_timeout_;       /** @brief Read timeout in ms */
  int32_t write_timeout_;      /** @brief Write timeout in ms */
  std::string ip_;             /** @brief IP of the device used as input */
  int32_t port_; /** @brief socket port of the device used as input */
  std::vector<std::string>
      enabled_publishers_;     /** @brief Enabled ROS publishers */
  bool timestamp_source_gnss_; /** @brief Flag that indicates from where to take
                                  the time reference */
  double baseline_dir_offset_deg_;
  double baseline_dip_offset_deg_;
  double track_update_min_speed_mps_;
};
