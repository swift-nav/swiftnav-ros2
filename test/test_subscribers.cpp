#include <array>
#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

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

constexpr uint32_t MAX_MSG_SIZE = sizeof(sbp_msg_t) + 8;
constexpr uint32_t IMU_RAW_SIZE = 25U;
constexpr uint32_t ODOM_MSG_SIZE = 17U;

/**
 * @brief Class to Mock a data source
 */
class MockedDataSource : public SbpDataSource {
 public:
  s32 write(const u8* buffer, u32 buffer_length) override {
    std::unique_lock<std::mutex> lock(buffer_lock_);
    memcpy(buffer_.data() + size_, buffer, buffer_length);
    size_ += buffer_length;
    return buffer_length;
  }

  void getBufferPayload(std::array<uint8_t, MAX_MSG_SIZE>& buffer) {
    std::unique_lock<std::mutex> lock(buffer_lock_);
    std::copy(std::begin(buffer_) + 6, std::end(buffer_), std::begin(buffer));
  }

  uint32_t getBufferSize() {
    std::unique_lock<std::mutex> lock(buffer_lock_);
    return size_;
  }

  void resetBuffer() {
    std::unique_lock<std::mutex> lock(buffer_lock_);
    buffer_.fill(0);
    size_ = 0U;
    printf("\n");
  }

 private:
  std::array<uint8_t, MAX_MSG_SIZE> buffer_;
  uint32_t size_{0};
  std::mutex buffer_lock_;
};

class SBPRunner {
 public:
  SBPRunner() {
    state_.set_reader(&data_source_);
    state_.set_writer(&data_source_);
    sbp_th_ = std::thread(&SBPRunner::sbp_thread_proc, this);
  }

  ~SBPRunner() {
    exit_requested_ = true;
    sbp_th_.join();
  }

  sbp::State* getState() { return &state_; }
  MockedDataSource* getDataSource() { return &data_source_; }

 private:
  void sbp_thread_proc() {
    while (!exit_requested_) {
      state_.process();
    }
  }

  std::thread sbp_th_;
  bool exit_requested_{false};
  sbp::State state_;
  MockedDataSource data_source_;
};

class TestROS2toSBP : public ::testing::Test {
 public:
  static void SetUpTestCase() {
    rclcpp::init(0, nullptr);
    logger_ = std::make_shared<MockedLogger>();
  }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  static LoggerPtr logger_;
  static SBPRunner runner_;
};

static bool timedOut(const uint64_t start, const uint64_t timeout) {
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  return (now - start) >= timeout;
}

LoggerPtr TestROS2toSBP::logger_;
SBPRunner TestROS2toSBP::runner_;

// *************************************************************************
// IMUSubscriber
TEST_F(TestROS2toSBP, DisabledIMUSubscriber) {
  // Node
  auto node = std::make_shared<rclcpp::Node>(IMU_NODE);

  // Disabled Subscriber
  IMUSubscriber subs(node.get(), runner_.getState(), IMU_TOPIC, false, logger_);

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
  while ((runner_.getDataSource()->getBufferSize() == 0U) &&
         !timedOut(start, 2 * SECONDS))
    executor.spin_once(std::chrono::nanoseconds(10000000LL));

  // Check timeout happened and no msg is received
  ASSERT_EQ(0U, runner_.getDataSource()->getBufferSize());
}

TEST_F(TestROS2toSBP, EnabledIMUSubscriber) {
  // Node
  auto node = std::make_shared<rclcpp::Node>(IMU_NODE);

  // Enabled Subscriber
  IMUSubscriber subs(node.get(), runner_.getState(), IMU_TOPIC, true, logger_);

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
  runner_.getDataSource()->resetBuffer();
  pub->publish(imu_msg);

  // Spin until a timeout or the message is published
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_once(std::chrono::milliseconds(0));

  const auto start =
      std::chrono::steady_clock::now().time_since_epoch().count();
  while ((runner_.getDataSource()->getBufferSize() < IMU_RAW_SIZE) &&
         !timedOut(start, 2 * SECONDS)) {
    executor.spin_once(std::chrono::nanoseconds(10000000LL));
  }

  // Check timeout happened and no msg is received
  ASSERT_EQ(IMU_RAW_SIZE, runner_.getDataSource()->getBufferSize());
  std::array<uint8_t, MAX_MSG_SIZE> buffer;
  runner_.getDataSource()->getBufferPayload(buffer);
  sbp_msg_t recv_msg;
  sbp_message_decode(buffer.data(), IMU_RAW_SIZE, nullptr, SbpMsgImuRaw,
                     &recv_msg);

  ASSERT_EQ(34, recv_msg.imu_raw.gyr_x);
  ASSERT_EQ(23, recv_msg.imu_raw.gyr_y);
  ASSERT_EQ(12, recv_msg.imu_raw.gyr_z);
}

// *************************************************************************
// OdometrySubscriber
TEST_F(TestROS2toSBP, DisabledOdometrySubscriber) {
  // Node
  auto node = std::make_shared<rclcpp::Node>(ODOMETRY_NODE);

  // Disabled Subscriber
  OdometrySubscriber subs(node.get(), runner_.getState(), ODOMETRY_TOPIC, false,
                          logger_);

  // Publisher
  auto pub = node->create_publisher<nav_msgs::msg::Odometry>(ODOMETRY_TOPIC, 1);

  // Create a ROS2 Msg
  nav_msgs::msg::Odometry odom_msg;

  odom_msg.pose.pose.position.x = 34;
  odom_msg.pose.pose.position.y = 23;
  odom_msg.pose.pose.position.z = 12;

  // Publish
  runner_.getDataSource()->resetBuffer();
  pub->publish(odom_msg);

  // Spin until a timeout or the message is published
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_once(std::chrono::milliseconds(0));

  const auto start =
      std::chrono::steady_clock::now().time_since_epoch().count();
  while ((runner_.getDataSource()->getBufferSize() == 0U) &&
         !timedOut(start, 2 * SECONDS))
    executor.spin_once(std::chrono::nanoseconds(10000000LL));

  // Check timeout happened and no msg is received
  ASSERT_EQ(0U, runner_.getDataSource()->getBufferSize());
}

TEST_F(TestROS2toSBP, EnabledOdometrySubscriber) {
  // Node
  auto node = std::make_shared<rclcpp::Node>(ODOMETRY_NODE);

  // Disabled Subscriber
  OdometrySubscriber subs(node.get(), runner_.getState(), ODOMETRY_TOPIC, true,
                          logger_);

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
  runner_.getDataSource()->resetBuffer();
  pub->publish(odom_msg);

  // Spin until a timeout or the message is published
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_once(std::chrono::milliseconds(0));

  const auto start =
      std::chrono::steady_clock::now().time_since_epoch().count();
  while ((runner_.getDataSource()->getBufferSize() < ODOM_MSG_SIZE) &&
         !timedOut(start, 2 * SECONDS))
    executor.spin_once(std::chrono::nanoseconds(10000000LL));

  // Check timeout happened and no msg is received
  ASSERT_EQ(ODOM_MSG_SIZE, runner_.getDataSource()->getBufferSize());
  std::array<uint8_t, MAX_MSG_SIZE> buffer;
  runner_.getDataSource()->getBufferPayload(buffer);
  sbp_msg_t recv_msg;
  sbp_message_decode(buffer.data(), ODOM_MSG_SIZE, nullptr, SbpMsgOdometry,
                     &recv_msg);
  ASSERT_EQ(0, sbp_message_cmp(SbpMsgOdometry, &sbp_msg, &recv_msg));
}
