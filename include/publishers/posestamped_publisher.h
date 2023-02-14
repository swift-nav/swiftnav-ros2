/*
 * Copyright (C) 2010-2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#pragma once

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <libsbp/cpp/message_handler.h>
#include <libsbp/cpp/state.h>

#include <publishers/dummy_publisher.h>
#include <publishers/sbp2ros2_publisher.h>

/**
 * @brief Class that listens for sbp_msg_pos_ecef_t and sbp_msg_orient_quat_t,
 * publishing a geometry_msgs::msg::PoseStamped ros2 message.
 *
 */
class PoseStampedPublisher
    : public DummyPublisher,
      public SBP2ROS2Publisher<geometry_msgs::msg::PoseStamped,
                               sbp_msg_pos_ecef_t, sbp_msg_orient_euler_t> {
 public:
  PoseStampedPublisher() = delete;

  /**
   * @brief Construct a new Pose Stamped Publisher object
   *
   * @param state SBP State object
   * @param topic_name Name of the topic to publish a
   * geometry_msgs::msg::PoseStamped message
   * @param node ROS 2 node object
   */
  PoseStampedPublisher(sbp::State* state, const std::string& topic_name,
                       rclcpp::Node* node, const LoggerPtr& logger,
                       const std::string& frame);

  /**
   * @brief Handles a sbp_msg_pos_ecef_t message.
   *
   * @param sender_id Ignored
   * @param msg Incoming sbp_msg_pos_ecef_t
   */
  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_pos_ecef_t& msg);

  /**
   * @brief Handles a sbp_msg_orient_euler_t message.
   *
   * @param sender_id Ignored
   * @param msg Incoming sbp_msg_orient_euler_t
   */
  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_orient_euler_t& msg);

 protected:
  /**
   * @brief Checks that the Ros2 eometry_msgs::msg::PoseStamped is complete, if
   * so, it publishes it
   *
   */
  void publish() override;

 private:
  tf2Scalar microdegreesToRads(const int32_t microdegrees) const;

  u32 last_received_orient_euler_tow_{0};
  tf2::Quaternion quat_;

  static constexpr uint32_t MAX_ORIENT_EULER_TIME_DIFF_ = 2000;
};