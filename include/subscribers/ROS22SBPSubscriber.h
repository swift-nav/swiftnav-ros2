#pragma once

#include <string>
#include <functional>

#include <libsbp/cpp/state.h>
#include <logging/issue_logger.h>
#include <rclcpp/rclcpp.hpp>

using namespace std::placeholders;


class ROS22SBPSubscriber{
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
   ROS22SBPSubscriber(rclcpp::Node* node, sbp::State* state,
                     const bool enabled, const LoggerPtr& logger)
       : state_(state),
         node_(node),
         enabled_(enabled),
         logger_(logger) {}

   /**
    * @brief Method to enable/disable the publishing of the ROS topic
    *
    * @param val Can publish (true), don't publish (false)
    */
   void enable(const bool val) { enabled_ = val; }

  protected:
   /**
    * @brief Method to send message to connected Swift device
    */
   virtual s8 send_message(const sbp_msg_type_t msg_type, const sbp_msg_t& msg)
   {
    return state_->send_message(SBP_SENDER_ID, msg_type, msg);
   }

   sbp::State* state_;
   rclcpp::Node* node_; /** @brief ROS 2 node object */
   bool enabled_; /** @brief Flag that enables or disables the publishing */
   LoggerPtr logger_; /** @brief Logging facility */
};