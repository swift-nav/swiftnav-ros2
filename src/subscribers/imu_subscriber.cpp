#include <subscribers/imu_subscriber.h>
#include <cmath>

IMUSubscriber::IMUSubscriber(rclcpp::Node* node, sbp::State* state,
                             const std::string& topic_name,
                             const LoggerPtr& logger)
    : ROS22SBPSubscriber(node, state, logger),
      subscriber_(node_->create_subscription<sensor_msgs::msg::Imu>(
          topic_name, 10,
          std::bind(&IMUSubscriber::topic_callback, this, _1))) {}

void IMUSubscriber::topic_callback(const sensor_msgs::msg::Imu& msg) {
  // sbp_msg_imu_aux_t sbp_imu_aux_msg; // TODO how to fill this?
  sbp_msg_t sbp_msg;

  sbp_msg.imu_raw.tow = msg.header.stamp.sec * 1000;
  sbp_msg.imu_raw.tow_f = msg.header.stamp.sec * 1000;
  sbp_msg.imu_raw.acc_x =
      static_cast<s16>(std::round(msg.linear_acceleration.x));
  sbp_msg.imu_raw.acc_y =
      static_cast<s16>(std::round(msg.linear_acceleration.y));
  sbp_msg.imu_raw.acc_z =
      static_cast<s16>(std::round(msg.linear_acceleration.z));
  sbp_msg.imu_raw.gyr_x = static_cast<s16>(std::round(msg.angular_velocity.x));
  sbp_msg.imu_raw.gyr_y = static_cast<s16>(std::round(msg.angular_velocity.y));
  sbp_msg.imu_raw.gyr_z = static_cast<s16>(std::round(msg.angular_velocity.z));

  send_message(SbpMsgImuRaw, sbp_msg);
}
