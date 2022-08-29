#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <libsbp/cpp/state.h>
#include <libsbp/cpp/message_handler.h>

#include <string>

/**
 * @brief Template abstract base class for the publishers
 *
 * @tparam ROS2MsgType Type of ROS 2 message you want to publish
 * @tparam SBPMsgTypes Types of the SBP messages you want to merge into the ROS
 * 2 message
 */
template<typename ROS2MsgType, typename... SBPMsgTypes>
class SBP2ROS2Publisher : private sbp::MessageHandler<SBPMsgTypes...> {
  public:
   SBP2ROS2Publisher() = delete;

   /**
    * @brief Construct a new SBP2ROS2Publisher object
    *
    * @param state SBP State object
    * @param topic_name Name of the topic to publish in ROS
    * @param node ROS 2 node object
    * @param enabled Flag telling if the topic should be published (true) or not (false)
    * @param frame frame is the frame of reference reported by the satellite receiver, usually 
    *              the location of the antenna. This is a Euclidean frame relative to the vehicle, 
    *              not a reference ellipsoid.
    * 
    */
   SBP2ROS2Publisher(sbp::State* state, const std::string& topic_name,
                     rclcpp::Node* node, const bool enabled, const std::string& frame)
       : sbp::MessageHandler<SBPMsgTypes...>(state),
         node_(node),
         enabled_(enabled),
         frame_(frame) {
     publisher_ = node_->create_publisher<ROS2MsgType>(topic_name, 10);
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
   virtual void publish() = 0;

   ROS2MsgType msg_;               /** @brief ROS 2 message to publish */
   uint32_t composition_mask_{0U}; /** @brief Bitmask used to know when a ROS
                                      message is completo for publishing */
   std::shared_ptr<rclcpp::Publisher<ROS2MsgType>>
       publisher_;      /** @brief ROS 2 publisher */
   rclcpp::Node* node_; /** @brief ROS 2 node object */
   bool enabled_; /** @brief Flag that enables or disables the publishing */
   std::string frame_;
};