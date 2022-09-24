#pragma once

#include <libsbp/cpp/state.h>
#include <logging/issue_logger.h>
#include <publishers/dummy_publisher.h>
#include <rclcpp/rclcpp.hpp>

enum class Publishers {
  Invalid,          // Id to use in params.yaml list of enabled publishers
  AngularRate,      //  1
  BaselineHeading,  //  2
  GnssTimeOffset,   //  3
  GpsFix,           //  4
  ImuAux,           //  5
  ImuRaw,           //  6
  NavSatFix,        //  7
  Odometry,         //  8
  OrientEuler,      //  9
  OrientQuat,       // 10
  TimeReference,    // 11
  Wheeltick,        // 12
  PoseStamped,      // 13
};

/**
 * @brief Function that creates a new publisher
 *
 * @param pub_type Type of the publisher you want to create (Publishers enum)
 * @param state SBP state object
 * @param topic_name Name of the topic to publish to
 * @param node ROS2 node
 * @param logger Logging facility
 * @param frame frame is the frame of reference reported by the satellite
 * receiver, usually the location of the antenna. This is a Euclidean frame
 * relative to the vehicle, not a reference ellipsoid.
 * @return Newly created publisher
 */
PublisherPtr publisherFactory(const Publishers pub_type, sbp::State* state,
                              const std::string& topic_name, rclcpp::Node* node,
                              const LoggerPtr& logger,
                              const std::string& frame);
