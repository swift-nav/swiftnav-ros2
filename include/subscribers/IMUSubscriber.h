#pragma once

#include <rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/imu.hpp>

#include<subscribers/ROS22SBPSubscriber.h>

class IMUSubscriber: public ROS22SBPSubscriber<sensor_msgs::msg::Imu>{
    public:
     IMUSubscriber() = delete;

     IMUSubscriber(rclcpp::Node* node, sbp::IWriter* writer,  const std::string& topic_name,
                        , const bool enabled); 
    
    protected:
     virtual void topic_callback(const sensor_msgs::msg::Imu & msg);
}