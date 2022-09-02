#pragma once

#include <rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/imu.hpp>

#include<subscribers/ROS22SBPSubscriber.h>

class IMUSubscriber : public ROS22SBPSubscriber{
    public:
     IMUSubscriber() = delete;

     IMUSubscriber(rclcpp::Node* node, sbp::State* state,
                   const std::string& topic_name, const bool enabled,
                   const LoggerPtr& logger);

    protected:
     virtual void topic_callback(const sensor_msgs::msg::Imu & msg);
    
     rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;   /** @brief ROS 2 publisher */
};