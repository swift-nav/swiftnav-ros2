#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <subscribers/dummy_subscriber.h>
#include <subscribers/ros2_2_sbp_subscriber.h>

class OdometrySubscriber : public DummySubscriber, public ROS22SBPSubscriber {
 public:
  OdometrySubscriber() = delete;

  OdometrySubscriber(rclcpp::Node* node, sbp::State* state,
                     const std::string& topic_name, const LoggerPtr& logger);

 protected:
  virtual void topic_callback(const nav_msgs::msg::Odometry& msg);

 private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      subscriber_; /** @brief ROS 2 subscriber */
};
