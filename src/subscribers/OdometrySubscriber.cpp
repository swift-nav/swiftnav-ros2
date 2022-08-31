#include<subscribers/OdometrySubscriber.h>

OdometrySubscriber::OdometrySubscriber(rclcpp::Node* node, sbp::State* state,
                                       const std::string& topic_name,
                                       const bool enabled,
                                       const LoggerPtr& logger)
    : ROS22SBPSubscriber(node, state, topic_name, enabled, logger) {}

void OdometrySubscriber::topic_callback(const nav_msgs::msg::Odometry & msg)
{
    sbp_msg_t sbp_msg;
    sbp_msg.odometry.tow = msg.header.stamp.sec * 1000; // + msg.header.stamp.nsec/10;
    sbp_msg.odometry.velocity = msg.twist.twist.linear.x * 1000;
    sbp_msg.odometry.flags = 2;

    if (enabled_) send_message(SbpMsgOdometry, sbp_msg);
}