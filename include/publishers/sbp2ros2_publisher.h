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

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <libsbp/cpp/message_handler.h>
#include <libsbp/cpp/state.h>

#include <logging/issue_logger.h>

#include <utils/config.h>
#include <string_view>

/**
 * @brief Template abstract base class for the publishers
 *
 * @tparam ROS2MsgType Type of ROS 2 message you want to publish
 * @tparam SBPMsgTypes Types of the SBP messages you want to merge into the ROS
 * 2 message
 */
template <typename ROS2MsgType, typename... SBPMsgTypes>
class SBP2ROS2Publisher : private sbp::MessageHandler<SBPMsgTypes...> {
 public:
  SBP2ROS2Publisher() = delete;

  /**
   * @brief Construct a new SBP2ROS2Publisher object
   *
   * @param state SBP State object
   * @param topic_name Name of the topic to publish in ROS
   * @param node ROS 2 node object
   * @param frame frame is the frame of reference reported by the satellite
   * receiver, usually the location of the antenna. This is a Euclidean frame
   * relative to the vehicle, not a reference ellipsoid.
   *
   */
  SBP2ROS2Publisher(sbp::State* state, const std::string_view topic_name,
                    rclcpp::Node* node, const LoggerPtr& logger,
                    const std::string& frame,
                    const std::shared_ptr<Config>& config)
      : sbp::MessageHandler<SBPMsgTypes...>(state),
        node_(node),
        frame_(frame),
        logger_(logger),
        config_(config) {
    publisher_ =
        node_->create_publisher<ROS2MsgType>(std::string(topic_name), 10);
  }

 protected:
  /**
   * @brief Method to publish the topic
   */
  virtual void publish() = 0;

  ROS2MsgType msg_;               /** @brief ROS 2 message to publish */
  uint32_t composition_mask_{0U}; /** @brief Bitmask used to know when a ROS
                                     message is completo for publishing */
  std::shared_ptr<rclcpp::Publisher<ROS2MsgType>>
      publisher_;      /** @brief ROS 2 publisher */
  rclcpp::Node* node_; /** @brief ROS 2 node object */
  std::string frame_;
  LoggerPtr logger_; /** @brief Logging facility */
  std::shared_ptr<Config> config_; /** Node configuration */
};