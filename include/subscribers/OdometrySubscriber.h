#pragma once

#include <rclcpp/rclcpp.hpp>
#include<nav_msgs/msg/odometry.hpp>

#include<subscribers/ROS22SBPSubscriber.h>

class OdometrySubscriber : public ROS22SBPSubscriber{
    public:
     OdometrySubscriber() = delete;

     OdometrySubscriber(rclcpp::Node* node, sbp::State* state,
                        const std::string& topic_name, const bool enabled,
                        const LoggerPtr& logger);

    protected:
     virtual void topic_callback(const nav_msgs::msg::Odometry & msg);

    private:
     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
         subscriber_; /** @brief ROS 2 subscriber */
};
