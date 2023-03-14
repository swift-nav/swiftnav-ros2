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
#include <functional>

#include <libsbp/cpp/state.h>
#include <logging/issue_logger.h>
#include <rclcpp/rclcpp.hpp>

using namespace std::placeholders;


class ROS22SBPSubscriber{
  public:
   ROS22SBPSubscriber() = delete;

   /**
    * @brief Construct a new ROS22SBPSubscriber object
    *
    * @param topic_name Name of the topic to publish in ROS
    * @param node ROS 2 node object
    * @param enabled Flag telling if the topic should be published (true) or not
    * (false)
    */
   ROS22SBPSubscriber(rclcpp::Node* node, sbp::State* state,
                      const LoggerPtr& logger)
       : state_(state), node_(node), logger_(logger) {}

  protected:
   /**
    * @brief Method to send message to connected Swift device
    */
   virtual s8 send_message(const sbp_msg_type_t msg_type, const sbp_msg_t& msg)
   {
    return state_->send_message(SBP_SENDER_ID, msg_type, msg);
   }

   sbp::State* state_;
   rclcpp::Node* node_; /** @brief ROS 2 node object */
   LoggerPtr logger_; /** @brief Logging facility */
};