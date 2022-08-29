#pragma once

#include <string>
#include <functional>

#include <libsbp/cpp/state.h>
#include <rclcpp/rclcpp.hpp>

using namespace std::placeholders;

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
   ROS22SBPSubscriber(rclcpp::Node* node, sbp::State* state, const std::string& topic_name,
                      const bool enabled)
       : state_(state),
         node_(node),
         subscriber_(node_->create_subscription<ROS2MsgType>(
          topic_name, 10, std::bind(&ROS22SBPSubscriber::topic_callback, this, _1))
         ),
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
   virtual s8 send_message(const sbp_msg_type_t msg_type, const sbp_msg_t& msg)
   {
    return state_->send_message(SBP_SENDER_ID, msg_type, msg);
   }

   sbp::State* state_;
   rclcpp::Node* node_; /** @brief ROS 2 node object */
   std::shared_ptr<rclcpp::Subscription<ROS2MsgType>> subscriber_;
   //rclcpp::Subscription<ROS2MsgType>::SharedPtr subscriber_;      /** @brief ROS 2 publisher */
   bool enabled_; /** @brief Flag that enables or disables the publishing */
};