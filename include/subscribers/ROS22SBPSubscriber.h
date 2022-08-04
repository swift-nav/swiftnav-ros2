#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <libsbp/cpp/state.h>


#include <string>

template<typename ROS2MsgType>
class ROS22SBPSubscriber {
  public:
   ROS22SBPSubscriber() = delete;

   /**
    * @brief Construct a new ROS22SBPSubscriber object
    *
    * @param topic_name Name of the topic to publish in ROS
    * @param node ROS 2 node object
    * @param enabled Flag telling if the topic should be published (true) or not
    * (false)
    */
   ROS22SBPSubscriber(rclcpp::Node* node, sbp::IWriter* writer,  const std::string& topic_name,
                     , const bool enabled)
       : node_(node),
         writer_(writer),
         subscriber_(node_->create_subscription<ROS2MsgType>(topic_name, 10, std::bind(&ROS22SBPSubscriber::topic_callback, this, _1))),
         enabled_(enabled)
   {
   }

   /**
    * @brief Method to enable/disable the publishing of the ROS topic
    *
    * @param val Can publish (true), don't publish (false)
    */
   void enable(const bool val) { enabled_ = val; }

  protected:
   /**
    * @brief Method to publish the topic
    */
   virtual void topic_callback(const ROS2MsgType & msg) = 0;

   rclcpp::Node* node_; /** @brief ROS 2 node object */
   sbp::IWriter* writer_;
   rclcpp::Subscription<ROS2MsgType>::SharedPtr subscriber_;      /** @brief ROS 2 publisher */
   bool enabled_; /** @brief Flag that enables or disables the publishing */
};