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

#include <utils/config.h>
#include <rclcpp/rclcpp.hpp>

Config::Config(rclcpp::Node* node) {
  declareParameters(node);
  loadParameters(node);
}

void Config::declareParameters(rclcpp::Node* node) {
  node->declare_parameter<std::string>("frame_name", "swiftnav-gnss");
  node->declare_parameter<bool>("log_sbp_messages", false);
  node->declare_parameter<std::string>("log_sbp_filepath", "");
  node->declare_parameter<int32_t>("interface", 0);
  node->declare_parameter<std::string>("sbp_file", "");
  node->declare_parameter<std::string>("device_name", "");
  node->declare_parameter<std::string>("connection_str", "");
  node->declare_parameter<int32_t>("read_timeout", 0);
  node->declare_parameter<int32_t>("write_timeout", 0);
  node->declare_parameter<std::string>("host_ip", "");
  node->declare_parameter<int32_t>("host_port", 0);
  node->declare_parameter<bool>("timestamp_source_gnss", true);
#if defined(FOUND_NEWER)
  node->declare_parameter("enabled_publishers", rclcpp::PARAMETER_STRING_ARRAY);
#else
  node->declare_parameter("enabled_publishers");
#endif
  node->declare_parameter<double>("baseline_dir_offset_deg", 0.0);
  node->declare_parameter<double>("baseline_dip_offset_deg", 0.0);
  node->declare_parameter<double>("track_update_min_speed_mps", 0.2);
}

void Config::loadParameters(rclcpp::Node* node) {
  node->get_parameter<std::string>("frame_name", frame_);
  node->get_parameter<bool>("log_sbp_messages", log_sbp_messages_);
  node->get_parameter<std::string>("log_sbp_filepath", log_path_);
  node->get_parameter<int32_t>("interface", interface_);
  node->get_parameter<std::string>("sbp_file", file_);
  node->get_parameter<std::string>("device_name", device_);
  node->get_parameter<std::string>("connection_str", connection_str_);
  node->get_parameter<int32_t>("read_timeout", read_timeout_);
  node->get_parameter<int32_t>("write_timeout", write_timeout_);
  node->get_parameter<std::string>("host_ip", ip_);
  node->get_parameter<int32_t>("host_port", port_);
  node->get_parameter<bool>("timestamp_source_gnss", timestamp_source_gnss_);
  enabled_publishers_ =
      node->get_parameter("enabled_publishers").as_string_array();
  node->get_parameter<double>("baseline_dir_offset_deg",
                              baseline_dir_offset_deg_);
  node->get_parameter<double>("baseline_dip_offset_deg",
                              baseline_dip_offset_deg_);
  node->get_parameter<double>("track_update_min_speed_mps",
                              track_update_min_speed_mps_);
}
