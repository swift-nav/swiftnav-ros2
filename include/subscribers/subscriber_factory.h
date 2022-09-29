#pragma once

#include <libsbp/cpp/state.h>
#include <logging/issue_logger.h>
#include <subscribers/dummy_subscriber.h>
#include <rclcpp/rclcpp.hpp>

enum class Subscribers {
  Invalid,   // Id to use in params.yaml list of enabled subscribers
  Imu,       //  1
  Odometry,  //  2
};

/**
 * @brief Function that creates a new subscriber
 *
 * @param sub_type Type of the subscriber you want to create (Subscribers enum)
 * @param state SBP state object
 * @param topic_name Name of the topic to subscribe to
 * @param node ROS2 node
 * @param logger Logging facility
 * @return Newly created subscriber
 */
SubscriberPtr subscriberFactory(const Subscribers sub_type, sbp::State* state,
                                const std::string& topic_name,
                                rclcpp::Node* node, const LoggerPtr& logger);
