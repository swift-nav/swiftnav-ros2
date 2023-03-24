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

#include <data_sources/sbp_data_sources.h>
#include <utils/config.h>
#include <utils/utils.h>

static const int64_t LOG_REPUBLISH_DELAY =
    TimeUtils::secondsToNanoseconds(2ULL);

/**
 * @brief Class that represents the ROS 2 driver node
 */
class SBPROS2DriverNode : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new SBPROS2DriverNode object
   */
  SBPROS2DriverNode() : Node("swiftnav_ros2_driver") {
    config_ = std::make_shared<Config>(this);
    logger_ = std::make_shared<ROSLogger>(LOG_REPUBLISH_DELAY);

    createDataSources();
    if (!data_source_) exit(EXIT_FAILURE);
    state_.set_reader(data_source_.get());
    state_.set_writer(data_source_.get());
    createPublishers();

    sbptoros2_ = std::make_shared<SBPToROS2Logger>(
        &state_, logger_, config_->getLogSBPMessages(), config_->getLogPath());

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
    data_source_ = dataSourceFactory(config_, logger_);
  }

  /**
   * @brief Method for creating the SBP to ROS2 publishers
   */
  void createPublishers() {
    auto frame = config_->getFrame();
    const auto publishers = config_->getPublishers();

    LOG_INFO(logger_, "Creating %u publishers", publishers.size());
    for (const auto& publisher : publishers) {
      LOG_INFO(logger_, "Adding publisher %s", publisher.c_str());
      pubs_manager_.add(
          publisherFactory(publisher, &state_, this, logger_, frame, config_));
    }
  }

  sbp::State state_;           /** @brief SBP state object */
  std::thread sbp_thread_;     /** @brief SBP messages processing thread */
  bool exit_requested_{false}; /** @brief Thread stopping flag */
  std::shared_ptr<Config> config_;             /** @brief Node configuration */
  std::shared_ptr<SbpDataSource> data_source_; /** @brief data source object */
  std::shared_ptr<ROSLogger> logger_; /** @brief ROS 2 logging object */
  PublisherManager
      pubs_manager_; /** @brief Manager for all the active publishers */
  std::shared_ptr<SBPToROS2Logger>
      sbptoros2_; /** @brief SBP to ROS2 logging object */
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SBPROS2DriverNode>());
  rclcpp::shutdown();

  return 0;
}
