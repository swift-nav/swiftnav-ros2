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

#include <publishers/publisher_factory.h>

#include <publishers/baseline_publisher.h>
#include <publishers/gpsfix_publisher.h>
#include <publishers/navsatfix_publisher.h>
#include <publishers/timereference_publisher.h>
#include <publishers/imu_publisher.h>


PublisherPtr publisherFactory(const Publishers pub_type, sbp::State* state,
                              const std::string& topic_name, rclcpp::Node* node,
                              const LoggerPtr& logger, const std::string& frame,
                              const std::shared_ptr<Config>& config) {
  PublisherPtr pub;

  switch (pub_type) {
    case Publishers::BaselineHeading:
      pub = std::make_shared<BaselinePublisher>(state, topic_name, node, logger,
                                                frame, config);
      break;

    case Publishers::GpsFix:
      pub = std::make_shared<GPSFixPublisher>(state, topic_name, node, logger,
                                              frame, config);
      break;

    case Publishers::NavSatFix:
      pub = std::make_shared<NavSatFixPublisher>(state, topic_name, node,
                                                 logger, frame, config);
      break;

    case Publishers::TimeReference:
      pub = std::make_shared<TimeReferencePublisher>(state, topic_name, node,
                                                     logger, frame, config);
      break;

    case Publishers::Imu:
      pub = std::make_shared<ImuPublisher>(state, topic_name, node,
                                           logger, frame, config);
      break;

    default:
      LOG_ERROR(logger, "Publisher id: %d isn't valid",
                static_cast<int>(pub_type));
      break;
  }

  return pub;
}
