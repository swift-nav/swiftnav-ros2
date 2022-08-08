#pragma once

#include <rclcpp/rclcpp.hpp>
#include<nav_msgs/msg/odometry.hpp>

#include<subscribers/ROS22SBPSubscriber.h>

class OdometrySubscriber : public ROS22SBPSubscriber<nav_msgs::msg::Odometry>{
    public:
     OdometrySubscriber() = delete;

     OdometrySubscriber(rclcpp::Node* node, sbp::State* state,  const std::string& topic_name,
                        const bool enabled); 
    
    protected:
     virtual void topic_callback(const nav_msgs::msg::Odometry & msg);
};