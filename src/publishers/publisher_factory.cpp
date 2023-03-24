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
#include <publishers/imu_publisher.h>
#include <publishers/navsatfix_publisher.h>
#include <publishers/twistwithcovariancestamped_publisher.h>
#include <publishers/timereference_publisher.h>
#include <string_view>

enum class Publishers {
  Invalid,
  GpsFix,                     // 1
  NavSatFix,                  // 2
  TwistWithCovarianceStamped, // 3
  Baseline,                   // 4
  TimeReference,              // 5
  Imu,                        // 6
};

struct PublisherMap {
  Publishers id;
  std::string_view name;
};

static const PublisherMap publishers[] = {
    {Publishers::GpsFix, "gpsfix"},
    {Publishers::NavSatFix, "navsatfix"},
    {Publishers::TwistWithCovarianceStamped, "twistwithcovariancestamped"},
    {Publishers::Baseline, "baseline"},
    {Publishers::TimeReference, "timereference"},
    {Publishers::Imu, "imu"},
};

PublisherPtr publisherFactory(const std::string& pub_type, sbp::State* state,
                              rclcpp::Node* node, const LoggerPtr& logger,
                              const std::string& frame,
                              const std::shared_ptr<Config>& config) {
  PublisherPtr pub;
  Publishers pub_id = Publishers::Invalid;
  std::string_view topic;

  for (const auto& publisher : publishers) {
    if (publisher.name == pub_type) {
      pub_id = publisher.id;
      topic = publisher.name;
    }
  }

  switch (pub_id) {
    case Publishers::Baseline:
      pub = std::make_shared<BaselinePublisher>(state, topic, node, logger,
                                                frame, config);
      break;

    case Publishers::GpsFix:
      pub = std::make_shared<GPSFixPublisher>(state, topic, node, logger, frame,
                                              config);
      break;

    case Publishers::NavSatFix:
      pub = std::make_shared<NavSatFixPublisher>(state, topic, node, logger,
                                                 frame, config);
      break;

    case Publishers::TwistWithCovarianceStamped:
      pub = std::make_shared<TwistWithCovarianceStampedPublisher>(state, topic, node, logger,
                                                 frame, config);
      break;

    case Publishers::TimeReference:
      pub = std::make_shared<TimeReferencePublisher>(state, topic, node, logger,
                                                     frame, config);
      break;

    case Publishers::Imu:
      pub = std::make_shared<ImuPublisher>(state, topic, node, logger, frame,
                                           config);
      break;

    default:
      LOG_ERROR(logger, "Publisher %s: isn't valid", pub_type.c_str());
      break;
  }

  return pub;
}
