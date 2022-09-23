#include<subscribers/IMUSubscriber.h>

IMUSubscriber::IMUSubscriber(rclcpp::Node* node, sbp::State* state,
                             const std::string& topic_name, const bool enabled,
                             const LoggerPtr& logger)
    : ROS22SBPSubscriber(node, state, enabled, logger),
      subscriber_(node_->create_subscription<sensor_msgs::msg::Imu>(
            topic_name, 10 ,std::bind(&IMUSubscriber::topic_callback, this, _1)))
      {}

void IMUSubscriber::topic_callback(const sensor_msgs::msg::Imu & msg)
{
  // sbp_msg_imu_aux_t sbp_imu_aux_msg; // TODO how to fill this?
  sbp_msg_t sbp_msg;

  sbp_msg.imu_raw.tow =
      msg.header.stamp.sec * 1000;  //+ msg.header.stamp.nsec / 10;
  sbp_msg.imu_raw.tow_f =
      msg.header.stamp.sec * 1000;  //+ msg.header.stamp.nsec / 10;
  sbp_msg.imu_raw.acc_x = msg.linear_acceleration.x;
  sbp_msg.imu_raw.acc_y = msg.linear_acceleration.y;
  sbp_msg.imu_raw.acc_z = msg.linear_acceleration.z;
  sbp_msg.imu_raw.gyr_x = msg.angular_velocity.x;
  sbp_msg.imu_raw.gyr_y = msg.angular_velocity.y;
  sbp_msg.imu_raw.gyr_z = msg.angular_velocity.z;

  if (enabled_) send_message(SbpMsgImuRaw, sbp_msg);
}
