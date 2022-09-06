#include <array>
#include <chrono>
#include <cstring>
#include <iostream>
#include <string>

#include <rclcpp/exceptions.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <libsbp/cpp/state.h>

#include <data_sources/sbp_data_sources.h>
#include <subscribers/IMUSubscriber.h>
#include <subscribers/OdometrySubscriber.h>

#include<test/mocked_logger.h>

const std::string IMU_TOPIC = "/imudata";
const std::string IMU_NODE = "IMUTester";
const std::string ODOMETRY_TOPIC = "/odomdata";
const std::string ODOMETRY_NODE = "ODOMTESTER";

constexpr uint64_t SECONDS = 1000000000ULL;

/**
 * @brief Class to Mock a data source
 */
class MockedDataSource : public SbpDataSource {
 public:
  s32 write(const u8* buffer, u32 buffer_length) override {
    memcpy(buffer_.data(), buffer, buffer_length);
    size_ = buffer_length;
    return buffer_length;
  }

  void getBuffer(std::array<uint8_t, sizeof(sbp_msg_t)>& buffer) const {
    buffer = buffer_;
  }

  uint32_t getBufferSize() const { return size_; }

  void resetBuffer() {
    buffer_.fill(0);
    size_ = 0U;
  }

 private:
  std::array<uint8_t, sizeof(sbp_msg_t)> buffer_;
  uint32_t size_{0};
};

class TestROS2toSBP : public ::testing::Test {
 public:
  static void SetUpTestCase() {
    rclcpp::init(0, nullptr);
    logger_ = std::make_shared<MockedLogger>();
    state_.set_writer(&data_source_);
  }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  static sbp::State state_;
  static LoggerPtr logger_;
  static MockedDataSource data_source_;
};

static bool timedOut(const uint64_t start, const uint64_t timeout) {
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  return (now - start) >= timeout;
}

sbp::State TestROS2toSBP::state_;
LoggerPtr TestROS2toSBP::logger_;
MockedDataSource TestROS2toSBP::data_source_;

// *************************************************************************
// IMUSubscriber
TEST_F(TestROS2toSBP, DisabledIMUSubscriber) {
  // Node
  auto node = std::make_shared<rclcpp::Node>(IMU_NODE);

  // Disabled Subscriber
  IMUSubscriber subs(node.get(), &state_, IMU_TOPIC, false, logger_);

  // Publisher
  auto pub = node->create_publisher<sensor_msgs::msg::Imu>(IMU_TOPIC, 1);

  // Create a ROS2 Msg
  sensor_msgs::msg::Imu imu_msg;

  imu_msg.angular_velocity.x = 34;
  imu_msg.angular_velocity.y = 23;
  imu_msg.angular_velocity.z = 12;

  // Publish
  pub->publish(imu_msg);

  // Spin until a timeout or the message is published
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_once(std::chrono::nanoseconds(0));

  const auto start =
      std::chrono::steady_clock::now().time_since_epoch().count();
  while ((data_source_.getBufferSize() == 0U) && !timedOut(start, 2 * SECONDS))
    executor.spin_once(std::chrono::nanoseconds(10000000LL));

  // Check timeout happened and no msg is received
  ASSERT_EQ(0U, data_source_.getBufferSize());
}

TEST_F(TestROS2toSBP, EnabledIMUSubscriber) {
  // Node
  auto node = std::make_shared<rclcpp::Node>(IMU_NODE);

  // Enabled Subscriber
  IMUSubscriber subs(node.get(), &state_, IMU_TOPIC, true, logger_);

  // Publisher
  auto pub = node->create_publisher<sensor_msgs::msg::Imu>(IMU_TOPIC, 1);

  // Create a ROS2 Msg
  sensor_msgs::msg::Imu imu_msg;

  imu_msg.angular_velocity.x = 34;
  imu_msg.angular_velocity.y = 23;
  imu_msg.angular_velocity.z = 12;

  // Create the equivalent SBP message
  sbp_msg_t sbp_msg;

  memset(&sbp_msg, 0, sizeof(sbp_msg));
  sbp_msg.imu_raw.gyr_x = 34;
  sbp_msg.imu_raw.gyr_y = 23;
  sbp_msg.imu_raw.gyr_z = 12;

  // Publish
  pub->publish(imu_msg);

  // Spin until a timeout or the message is published
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_once(std::chrono::milliseconds(0));

  const auto start =
      std::chrono::steady_clock::now().time_since_epoch().count();
  while ((data_source_.getBufferSize() == 0U) && !timedOut(start, 2 * SECONDS))
    executor.spin_once(std::chrono::nanoseconds(10000000LL));

  // Check timeout happened and no msg is received
  ASSERT_NE(0U, data_source_.getBufferSize());
  std::array<uint8_t, sizeof(sbp_msg_t)> buffer;
  data_source_.getBuffer(buffer);
  const uint32_t size = data_source_.getBufferSize();
  ASSERT_EQ(0, memcmp(&sbp_msg.imu_raw, buffer.data(), size));
}

// *************************************************************************
// OdometrySubscriber
TEST_F(TestROS2toSBP, DisabledOdometrySubscriber) {
  // Node
  auto node = std::make_shared<rclcpp::Node>(ODOMETRY_NODE);

  // Disabled Subscriber
  OdometrySubscriber subs(node.get(), &state_, ODOMETRY_TOPIC, false, logger_);

  // Publisher
  auto pub = node->create_publisher<nav_msgs::msg::Odometry>(ODOMETRY_TOPIC, 1);

  // Create a ROS2 Msg
  nav_msgs::msg::Odometry odom_msg;

  odom_msg.pose.pose.position.x = 34;
  odom_msg.pose.pose.position.y = 23;
  odom_msg.pose.pose.position.z = 12;

  // Publish
  pub->publish(odom_msg);

  // Spin until a timeout or the message is published
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_once(std::chrono::milliseconds(0));

  const auto start =
      std::chrono::steady_clock::now().time_since_epoch().count();
  while ((data_source_.getBufferSize() == 0U) && !timedOut(start, 2 * SECONDS))
    executor.spin_once(std::chrono::nanoseconds(10000000LL));

  // Check timeout happened and no msg is received
  ASSERT_EQ(0U, data_source_.getBufferSize());
}

TEST_F(TestROS2toSBP, EnabledOdometrySubscriber) {
  // Node
  auto node = std::make_shared<rclcpp::Node>(ODOMETRY_NODE);

  // Disabled Subscriber
  OdometrySubscriber subs(node.get(), &state_, ODOMETRY_TOPIC, false, logger_);

  // Publisher
  auto pub = node->create_publisher<nav_msgs::msg::Odometry>(ODOMETRY_TOPIC, 1);

  // Create a ROS2 Msg
  nav_msgs::msg::Odometry odom_msg;

  odom_msg.pose.pose.position.x = 34;
  odom_msg.pose.pose.position.y = 23;
  odom_msg.pose.pose.position.z = 12;

  // Create the equivalent SBP message
  sbp_msg_t sbp_msg;

  sbp_msg.odometry.tow =
      odom_msg.header.stamp.sec * 1000;  // + msg.header.stamp.nsec/10;
  sbp_msg.odometry.velocity = odom_msg.twist.twist.linear.x * 1000;
  sbp_msg.odometry.flags = 2;

  // Publish
  pub->publish(odom_msg);

  // Spin until a timeout or the message is published
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_once(std::chrono::milliseconds(0));

  const auto start =
      std::chrono::steady_clock::now().time_since_epoch().count();
  while ((data_source_.getBufferSize() == 0U) && !timedOut(start, 2 * SECONDS))
    executor.spin_once(std::chrono::nanoseconds(10000000LL));

  // Check timeout happened and no msg is received
  ASSERT_NE(0U, data_source_.getBufferSize());
  std::array<uint8_t, sizeof(sbp_msg_t)> buffer;
  data_source_.getBuffer(buffer);
  const uint32_t size = data_source_.getBufferSize();
  ASSERT_EQ(0, memcmp(&sbp_msg.odometry, buffer.data(), size));
}
