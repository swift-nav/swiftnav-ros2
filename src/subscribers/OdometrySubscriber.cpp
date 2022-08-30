#include<subscribers/OdometrySubscriber.h>

OdometrySubscriber::OdometrySubscriber(rclcpp::Node* node, 
                             sbp::State* state,
                             const std::string& topic_name,
                             const bool enabled)
    : ROS22SBPSubscriber(node, state, topic_name, enabled)
{
}

void OdometrySubscriber::topic_callback(const nav_msgs::msg::Odometry & msg)
{
    sbp_msg_t sbp_msg;
    sbp_msg.odometry.tow = msg.header.stamp.sec * 1000; // + msg.header.stamp.nsec/10;
    sbp_msg.odometry.velocity = msg.twist.twist.linear.x * 1000;
    sbp_msg.odometry.flags = 2;

    sbp_msg_type_t msg_type = SbpMsgOdometry;

    this->send_message(msg_type, sbp_msg);
}