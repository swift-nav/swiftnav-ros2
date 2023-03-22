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

#include <libsbp/cpp/state.h>
#include <logging/issue_logger.h>
#include <publishers/base_publisher.h>
#include <utils/config.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

/**
 * @brief Function that creates a new publisher
 *
 * @param pub_type Type of the publisher you want to create
 * @param state SBP state object
 * @param node ROS2 node
 * @param logger Logging facility
 * @param frame frame is the frame of reference reported by the satellite
 * receiver, usually the location of the antenna. This is a Euclidean frame
 * relative to the vehicle, not a reference ellipsoid.
 * @return Newly created publisher
 */
PublisherPtr publisherFactory(const std::string& pub_type, sbp::State* state,
                              rclcpp::Node* node, const LoggerPtr& logger,
                              const std::string& frame,
                              const std::shared_ptr<Config>& config);
