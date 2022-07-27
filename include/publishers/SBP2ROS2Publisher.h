#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <libsbp/cpp/state.h>
#include <libsbp/cpp/message_handler.h>

template<typename ROS2MsgType, typename... SBPMsgTypes>
class SBP2ROS2Publisher : private sbp::MessageHandler<SBPMsgTypes...> {
  public:
   SBP2ROS2Publisher(sbp::State* state,
                     std::shared_ptr<rclcpp::Publisher<ROS2MsgType>> publisher,
                     rclcpp::Node* node, const bool enabled)
       : sbp::MessageHandler<SBPMsgTypes...>(state),
         publisher_(publisher),
         node_(node),
         enabled_(enabled) {}

   void enable(const bool val) { enabled_ = val; }

  protected:
   virtual void publish() = 0;

   bool enabled_{true};
   ROS2MsgType msg_;
   uint32_t composition_mask_{0U};
   std::shared_ptr<rclcpp::Publisher<ROS2MsgType>> publisher_;
   rclcpp::Node* node_;
};