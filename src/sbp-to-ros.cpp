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

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <logging/ros_logger.h>
#include <logging/sbp_to_ros2_logger.h>

#include <libsbp/cpp/message_handler.h>
#include <libsbp/cpp/state.h>

#include <publishers/publisher_factory.h>
#include <publishers/publisher_manager.h>

#include <subscribers/subscriber_factory.h>
#include <subscribers/subscriber_manager.h>

#include <data_sources/sbp_data_sources.h>
#include <utils.h>

static const int64_t LOG_REPUBLISH_DELAY = 2_ns;

/**
 * @brief Class that represents the ROS 2 driver node
 */
class SBPROS2DriverNode : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new SBPROS2DriverNode object
   */
  SBPROS2DriverNode() : Node("SBPRos2Driver") {
    declareParameters();
    get_parameter<std::string>("frame_name", frame_);
    logger_ = std::make_shared<ROSLogger>(LOG_REPUBLISH_DELAY);

    createDataSources();
    if (!data_source_) exit(EXIT_FAILURE);
    state_.set_reader(data_source_.get());
    state_.set_writer(data_source_.get());
    createPublishers();
    createSubscribers();

    bool log_sbp_messages;
    std::string log_path;

    get_parameter<bool>("log_sbp_messages", log_sbp_messages);
    get_parameter<std::string>("log_sbp_filepath", log_path);
    sbptoros2_ = std::make_shared<SBPToROS2Logger>(&state_, logger_,
                                                   log_sbp_messages, log_path);

    /* SBP Callback processing thread */
    sbp_thread_ = std::thread(&SBPROS2DriverNode::processSBP, this);
  }

  // Deleted methods
  SBPROS2DriverNode(const SBPROS2DriverNode&) = delete;
  SBPROS2DriverNode(SBPROS2DriverNode&&) = delete;
  SBPROS2DriverNode& operator=(const SBPROS2DriverNode&) = delete;
  SBPROS2DriverNode& operator=(SBPROS2DriverNode&&) = delete;

  /**
   * @brief Destroy the SBPROS2DriverNode object
   */
  ~SBPROS2DriverNode() {
    exit_requested_ = true;
    if (sbp_thread_.joinable()) sbp_thread_.join();
  }

  /**
   * @brief SBP messages processing thread
   */
  void processSBP() {
    while (!exit_requested_) {
      state_.process();
    }
  }

 private:
  /**
   * @brief Method for creating the data sources
   */
  void createDataSources() {
    int32_t interface;

    get_parameter<int32_t>("interface", interface);
    LOG_INFO(logger_, "Using interface type: %d", interface);

    switch (interface) {
      case FILE_DATA_SOURCE: {
        std::string file;
        get_parameter<std::string>("sbp_file", file);
        data_source_ = dataSourceFactory(file, logger_);
      } break;

      case SERIAL_DATA_SOURCE: {
        std::string device;
        std::string connection_str;
        int32_t read_timeout;
        int32_t write_timeout;
        get_parameter<std::string>("device_name", device);
        get_parameter<std::string>("connection_str", connection_str);
        get_parameter<int32_t>("read_timeout", read_timeout);
        get_parameter<int32_t>("write_timeout", write_timeout);
        data_source_ = dataSourceFactory(device, connection_str, read_timeout,
                                         write_timeout, logger_);
      } break;

      case TCP_DATA_SOURCE: {
        std::string ip;
        int32_t port;
        int32_t read_timeout;
        int32_t write_timeout;
        get_parameter<std::string>("host_ip", ip);
        get_parameter<int32_t>("host_port", port);
        get_parameter<int32_t>("read_timeout", read_timeout);
        get_parameter<int32_t>("write_timeout", write_timeout);
        data_source_ = dataSourceFactory(ip, static_cast<uint16_t>(port),
                                         read_timeout, write_timeout, logger_);
      } break;

      default:
        LOG_FATAL(logger_, "Could not create a data source of type: %d",
                  interface);
        break;
    }
  }

  /**
   * @brief ROS 2 parameters declaration methods
   */
  void declareParameters() {
    declare_parameter<int32_t>("interface", 0);
    declare_parameter<std::string>("sbp_file", "");
    declare_parameter<std::string>("device_name", "");
    declare_parameter<std::string>("connection_str", "");
    declare_parameter<std::string>("host_ip", "");
    declare_parameter<int32_t>("host_port", 0);
    declare_parameter<int32_t>("read_timeout", 0);
    declare_parameter<int32_t>("write_timeout", 0);
    declare_parameter("enabled_publishers_ids",
                      rclcpp::PARAMETER_INTEGER_ARRAY);
    declare_parameter("enabled_publishers_topics",
                      rclcpp::PARAMETER_STRING_ARRAY);
    declare_parameter("enabled_subscribers_ids",
                      rclcpp::PARAMETER_INTEGER_ARRAY);
    declare_parameter("enabled_subscribers_topics",
                      rclcpp::PARAMETER_STRING_ARRAY);
    declare_parameter<bool>("log_sbp_messages", false);
    declare_parameter<std::string>("log_sbp_filepath", "");
    declare_parameter<std::string>("frame_name", "swiftnav-gnss");
  }

  /**
   * @brief Method for creating the SBP to ROS2 publishers
   */
  void createPublishers() {
    const auto ids = get_parameter("enabled_publishers_ids").as_integer_array();
    const auto topics =
        get_parameter("enabled_publishers_topics").as_string_array();

    if (ids.size() != topics.size()) {
      LOG_FATAL(logger_, "Mistmached number of publishers ids and topics");
      exit(0);
    }

    LOG_INFO(logger_, "Creating %u publishers", ids.size());
    for (uint32_t i = 0; i < ids.size(); ++i) {
      if (ids[i] == 0) continue;
      LOG_INFO(logger_, "Adding publisher id: %d with topic: %s", ids[i],
               topics[i].c_str());
      pubs_manager_.add(publisherFactory(static_cast<Publishers>(ids[i]),
                                         &state_, topics[i], this, logger_,
                                         frame_));
    }
  }

  /**
   * @brief Method for creating ROS2 subscribers to SBP messages
   */
  void createSubscribers() {
    const auto ids =
        get_parameter("enabled_subscribers_ids").as_integer_array();
    const auto topics =
        get_parameter("enabled_subscribers_topics").as_string_array();

    if (ids.size() != topics.size()) {
      LOG_FATAL(logger_, "Mistmached number of subscribers ids and topics");
      exit(0);
    }

    LOG_INFO(logger_, "Creating %u subscribers", ids.size());
    for (uint32_t i = 0; i < ids.size(); ++i) {
      if (ids[i] == 0) continue;
      LOG_INFO(logger_, "Adding subscriber id: %d with topic: %s", ids[i],
               topics[i].c_str());
      subs_manager_.add(subscriberFactory(static_cast<Subscribers>(ids[i]),
                                          &state_, topics[i], this, logger_));
    }
  }

  sbp::State state_;           /** @brief SBP state object */
  std::thread sbp_thread_;     /** @brief SBP messages processing thread */
  bool exit_requested_{false}; /** @brief Thread stopping flag */
  std::shared_ptr<SbpDataSource> data_source_; /** @brief data source object */
  std::shared_ptr<ROSLogger> logger_; /** @brief ROS 2 logging object */
  PublisherManager
      pubs_manager_; /** @brief Manager for all the active publishers */
  SubscriberManager
      subs_manager_; /** @brief Manager for all the active subscribers */
  std::shared_ptr<SBPToROS2Logger>
      sbptoros2_; /** @brief SBP to ROS2 logging object */
  std::string frame_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SBPROS2DriverNode>());
  rclcpp::shutdown();

  return 0;
}
