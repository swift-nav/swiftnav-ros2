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

#include <subscribers/subscriber_factory.h>

#include <subscribers/imu_subscriber.h>
#include <subscribers/odometry_subscriber.h>

SubscriberPtr subscriberFactory(const Subscribers sub_type, sbp::State* state,
                                const std::string& topic_name,
                                rclcpp::Node* node, const LoggerPtr& logger) {
  SubscriberPtr sub;

  switch (sub_type) {
    case Subscribers::Imu:
      sub = std::make_shared<IMUSubscriber>(node, state, topic_name, logger);
      break;

    case Subscribers::Odometry:
      sub =
          std::make_shared<OdometrySubscriber>(node, state, topic_name, logger);
      break;

    default:
      LOG_ERROR(logger, "Subscriber id: %d isn't valid",
                static_cast<int>(sub_type));
      break;
  }

  return sub;
}
