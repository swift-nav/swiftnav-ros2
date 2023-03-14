/*
 * Copyright (C) 2010-2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <deque>
#include <mutex>
#include <thread>

#include <rclcpp/exceptions.hpp>
#include <rclcpp/rclcpp.hpp>

#include <test/mocked_logger.h>
#include <test/test_utils.h>

#include <data_sources/sbp_data_sources.h>
#include <publishers/publisher_factory.h>

#include <swiftnav_ros2_driver/msg/angular_rate.hpp>
#include <swiftnav_ros2_driver/msg/baseline_heading.hpp>
#include <swiftnav_ros2_driver/msg/gnss_time_offset.hpp>
#include <swiftnav_ros2_driver/msg/imu_aux.hpp>
#include <swiftnav_ros2_driver/msg/imu_raw.hpp>
#include <swiftnav_ros2_driver/msg/odometry.hpp>
#include <swiftnav_ros2_driver/msg/orient_euler.hpp>
#include <swiftnav_ros2_driver/msg/orient_quat.hpp>
#include <swiftnav_ros2_driver/msg/wheeltick.hpp>

constexpr uint32_t MAX_MSG_SIZE = 255;
constexpr uint64_t SECONDS = 1000000000ULL;

static bool timedOut(const uint64_t start, const uint64_t timeout) {
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  return (now - start) >= timeout;
}

/**
 * @brief Class to Mock a data source
 */
class MockedDataSource : public SbpDataSource {
 public:
  s32 read(u8* buffer, u32 buffer_length) override {
    s32 result = -1;

    std::unique_lock<std::mutex> lock(data_lock_);
    if (buffer_.size()) {
      const uint32_t to_read =
          std::min(buffer_.size(), static_cast<size_t>(buffer_length));
      for (uint32_t i = 0; i < to_read; ++i) {
        buffer[i] = buffer_.front();
        buffer_.pop_front();
      }

      result = to_read;
    }

    return result;
  }

  s32 write(const u8* buffer, u32 buffer_length) override {
    std::unique_lock<std::mutex> lock(data_lock_);
    std::copy(buffer, buffer + buffer_length, std::back_inserter(buffer_));
    return buffer_length;
  }

 private:
  std::deque<uint8_t> buffer_;
  std::mutex data_lock_;
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
  void inject(const sbp_msg_t& msg, const sbp_msg_type_t msg_type) {
    state_.send_message(SBP_SENDER_ID, msg_type, msg);
  }

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

class TestCustomPublishers : public ::testing::Test {
 public:
  static void SetUpTestCase() {
    rclcpp::init(0, nullptr);
    logger_ = std::make_shared<MockedLogger>();
  }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  template <typename rosT, typename sbpT, typename Func>
  void testPublisher(const Publishers pub_type, const sbpT& msg,
                     const sbp_msg_type_t msg_type, Func comp) {
    bool test_finished = false;
    bool timed_out = false;
    auto node = std::make_shared<rclcpp::Node>("TestCustomPublishersNode");
    auto pub = publisherFactory(pub_type, runner_.getState(), topic_name_,
                                node.get(), logger_, frame_name_);
    auto subs_call = [&msg, &test_finished, &comp](const rosT& ros_msg) {
      comp(msg, ros_msg);
      test_finished = true;
    };
    auto sub = node->create_subscription<rosT>(topic_name_, 1, subs_call);

    // publish
    runner_.inject(msg, msg_type);

    // wait for result
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin_once(std::chrono::milliseconds(0));

    const auto start =
        std::chrono::steady_clock::now().time_since_epoch().count();

    while (!test_finished && !timed_out) {
      executor.spin_once(std::chrono::nanoseconds(10000000LL));
      timed_out = timedOut(start, 2 * SECONDS);
    }

    ASSERT_FALSE(timed_out);
  }

  const std::string topic_name_ = "test_custom";
  const std::string frame_name_ = "test_frame";
  static LoggerPtr logger_;
  static SBPRunner runner_;
};

LoggerPtr TestCustomPublishers::logger_;
SBPRunner TestCustomPublishers::runner_;

TEST_F(TestCustomPublishers, CreateInvalidPublisher) {
  auto node = std::make_shared<rclcpp::Node>("TestCustomPublishersNode");
  auto pub = publisherFactory(static_cast<Publishers>(-1), runner_.getState(),
                              topic_name_, node.get(), logger_, frame_name_);
  ASSERT_FALSE(pub);
}

TEST_F(TestCustomPublishers, CreateAngularRatePublisher) {
  sbp_msg_t msg;

  msg.angular_rate.tow = 1234;
  msg.angular_rate.x = 10;
  msg.angular_rate.y = 20;
  msg.angular_rate.z = 30;
  msg.angular_rate.flags = 0x0C;

  auto check =
      [](const sbp_msg_t& msg,
         const swiftnav_ros2_driver::msg::AngularRate& ros_msg) -> void {
    ASSERT_EQ(msg.angular_rate.flags, ros_msg.flags);
    ASSERT_EQ(msg.angular_rate.tow, ros_msg.tow);
    ASSERT_EQ(msg.angular_rate.x, ros_msg.x);
    ASSERT_EQ(msg.angular_rate.y, ros_msg.y);
    ASSERT_EQ(msg.angular_rate.z, ros_msg.z);
  };

  testPublisher<swiftnav_ros2_driver::msg::AngularRate>(
      Publishers::AngularRate, msg, SbpMsgAngularRate, check);
}

TEST_F(TestCustomPublishers, CreateBaselineHeadingPublisher) {
#if 0 //!! TODO
  sbp_msg_t msg;

  msg.baseline_heading.flags = 0x0D;
  msg.baseline_heading.heading = 1780;
  msg.baseline_heading.n_sats = 3;
  msg.baseline_heading.tow = 3254;

  auto check =
      [](const sbp_msg_t& msg,
         const swiftnav_ros2_driver::msg::BaselineHeading& ros_msg) -> void {
    ASSERT_EQ(msg.baseline_heading.flags, ros_msg.flags);
    ASSERT_EQ(msg.baseline_heading.heading, ros_msg.heading);
    ASSERT_EQ(msg.baseline_heading.n_sats, ros_msg.n_sats);
    ASSERT_EQ(msg.baseline_heading.tow, ros_msg.tow);
  };

  testPublisher<swiftnav_ros2_driver::msg::BaselineHeading>(
      Publishers::BaselineHeading, msg, SbpMsgBaselineHeading, check);
#endif
}

TEST_F(TestCustomPublishers, CreateGnssTimeOffsetPublisher) {
  sbp_msg_t msg;

  msg.gnss_time_offset.flags = 0x1E;
  msg.gnss_time_offset.microseconds = 17;
  msg.gnss_time_offset.milliseconds = 3;
  msg.gnss_time_offset.weeks = 1;

  auto check =
      [](const sbp_msg_t& msg,
         const swiftnav_ros2_driver::msg::GnssTimeOffset& ros_msg) -> void {
    ASSERT_EQ(msg.gnss_time_offset.flags, ros_msg.flags);
    ASSERT_EQ(msg.gnss_time_offset.microseconds, ros_msg.microseconds);
    ASSERT_EQ(msg.gnss_time_offset.milliseconds, ros_msg.milliseconds);
    ASSERT_EQ(msg.gnss_time_offset.weeks, ros_msg.weeks);
  };

  testPublisher<swiftnav_ros2_driver::msg::GnssTimeOffset>(
      Publishers::GnssTimeOffset, msg, SbpMsgGnssTimeOffset, check);
}

TEST_F(TestCustomPublishers, CreateImuAuxPublisher) {
  sbp_msg_t msg;

  msg.imu_aux.imu_conf = 0x11;
  msg.imu_aux.imu_type = 2;
  msg.imu_aux.temp = 18;

  auto check = [](const sbp_msg_t& msg,
                  const swiftnav_ros2_driver::msg::ImuAux& ros_msg) -> void {
    ASSERT_EQ(msg.imu_aux.imu_conf, ros_msg.imu_conf);
    ASSERT_EQ(msg.imu_aux.imu_type, ros_msg.imu_type);
    ASSERT_EQ(msg.imu_aux.temp, ros_msg.temp);
  };

  testPublisher<swiftnav_ros2_driver::msg::ImuAux>(Publishers::ImuAux, msg,
                                                   SbpMsgImuAux, check);
}

TEST_F(TestCustomPublishers, CreateImuRawPublisher) {
  sbp_msg_t msg;

  msg.imu_raw.acc_x = 13;
  msg.imu_raw.acc_y = 1;
  msg.imu_raw.acc_z = -3;
  msg.imu_raw.gyr_x = 0;
  msg.imu_raw.gyr_y = -2;
  msg.imu_raw.gyr_z = 22;
  msg.imu_raw.tow = 73646;
  msg.imu_raw.tow_f = 12;

  auto check = [](const sbp_msg_t& msg,
                  const swiftnav_ros2_driver::msg::ImuRaw& ros_msg) -> void {
    ASSERT_EQ(msg.imu_raw.acc_x, ros_msg.acc_x);
    ASSERT_EQ(msg.imu_raw.acc_y, ros_msg.acc_y);
    ASSERT_EQ(msg.imu_raw.acc_z, ros_msg.acc_z);
    ASSERT_EQ(msg.imu_raw.gyr_x, ros_msg.gyr_x);
    ASSERT_EQ(msg.imu_raw.gyr_y, ros_msg.gyr_y);
    ASSERT_EQ(msg.imu_raw.gyr_z, ros_msg.gyr_z);
    ASSERT_EQ(msg.imu_raw.tow, ros_msg.tow);
    ASSERT_EQ(msg.imu_raw.tow_f, ros_msg.tow_f);
  };

  testPublisher<swiftnav_ros2_driver::msg::ImuRaw>(Publishers::ImuRaw, msg,
                                                   SbpMsgImuRaw, check);
}

TEST_F(TestCustomPublishers, CreateOdometryPublisher) {
  sbp_msg_t msg;

  msg.odometry.flags = 13;
  msg.odometry.velocity = 3343;
  msg.imu_raw.tow = 11223344;

  auto check = [](const sbp_msg_t& msg,
                  const swiftnav_ros2_driver::msg::Odometry& ros_msg) -> void {
    ASSERT_EQ(msg.odometry.flags, ros_msg.flags);
    ASSERT_EQ(msg.odometry.velocity, ros_msg.velocity);
    ASSERT_EQ(msg.imu_raw.tow, ros_msg.tow);
  };

  testPublisher<swiftnav_ros2_driver::msg::Odometry>(Publishers::Odometry, msg,
                                                     SbpMsgOdometry, check);
}

TEST_F(TestCustomPublishers, CreateOrientEulerPublisher) {
  sbp_msg_t msg;

  msg.orient_euler.flags = 4;
  msg.orient_euler.pitch = -300;
  msg.orient_euler.pitch_accuracy = 0.3;
  msg.orient_euler.roll = 64;
  msg.orient_euler.roll_accuracy = 0.92;
  msg.orient_euler.tow = 53442;
  msg.orient_euler.yaw = 112;
  msg.orient_euler.yaw_accuracy = 0.67;

  auto check =
      [](const sbp_msg_t& msg,
         const swiftnav_ros2_driver::msg::OrientEuler& ros_msg) -> void {
    ASSERT_EQ(msg.orient_euler.flags, ros_msg.flags);
    ASSERT_EQ(msg.orient_euler.pitch, ros_msg.pitch);
    ASSERT_TRUE(fabs(msg.orient_euler.pitch_accuracy - ros_msg.pitch_accuracy) <
                DBL_EPSILON);
    ASSERT_EQ(msg.orient_euler.roll, ros_msg.roll);
    ASSERT_TRUE(fabs(msg.orient_euler.roll_accuracy - ros_msg.roll_accuracy) <
                DBL_EPSILON);
    ASSERT_EQ(msg.orient_euler.yaw, ros_msg.yaw);
    ASSERT_TRUE(fabs(msg.orient_euler.yaw_accuracy - ros_msg.yaw_accuracy) <
                DBL_EPSILON);
    ASSERT_EQ(msg.orient_euler.tow, ros_msg.tow);
  };

  testPublisher<swiftnav_ros2_driver::msg::OrientEuler>(
      Publishers::OrientEuler, msg, SbpMsgOrientEuler, check);
}

TEST_F(TestCustomPublishers, CreateOrientQuatPublisher) {
  sbp_msg_t msg;

  msg.orient_quat.flags = 4;
  msg.orient_euler.tow = 53442;
  msg.orient_quat.w = 33;
  msg.orient_quat.w_accuracy = 0.9;
  msg.orient_quat.x = 1123;
  msg.orient_quat.x_accuracy = 0.92;
  msg.orient_quat.y = 112;
  msg.orient_quat.y_accuracy = 0.345;
  msg.orient_quat.z = 21;
  msg.orient_quat.z_accuracy = 0.81;

  auto check =
      [](const sbp_msg_t& msg,
         const swiftnav_ros2_driver::msg::OrientQuat& ros_msg) -> void {
    ASSERT_EQ(msg.orient_quat.flags, ros_msg.flags);
    ASSERT_EQ(msg.orient_quat.w, ros_msg.w);
    ASSERT_TRUE(fabs(msg.orient_quat.w_accuracy - ros_msg.w_accuracy) <
                DBL_EPSILON);
    ASSERT_EQ(msg.orient_quat.x, ros_msg.x);
    ASSERT_TRUE(fabs(msg.orient_quat.x_accuracy - ros_msg.x_accuracy) <
                DBL_EPSILON);
    ASSERT_EQ(msg.orient_quat.y, ros_msg.y);
    ASSERT_TRUE(fabs(msg.orient_quat.y_accuracy - ros_msg.y_accuracy) <
                DBL_EPSILON);
    ASSERT_EQ(msg.orient_quat.z, ros_msg.z);
    ASSERT_TRUE(fabs(msg.orient_quat.z_accuracy - ros_msg.z_accuracy) <
                DBL_EPSILON);
    ASSERT_EQ(msg.orient_quat.tow, ros_msg.tow);
  };

  testPublisher<swiftnav_ros2_driver::msg::OrientQuat>(
      Publishers::OrientQuat, msg, SbpMsgOrientQuat, check);
}

TEST_F(TestCustomPublishers, CreateWheeltickPublisher) {
  sbp_msg_t msg;

  msg.wheeltick.flags = 135;
  msg.wheeltick.source = 2;
  msg.wheeltick.ticks = 337454;
  msg.wheeltick.time = 78323498493;

  auto check = [](const sbp_msg_t& msg,
                  const swiftnav_ros2_driver::msg::Wheeltick& ros_msg) -> void {
    ASSERT_EQ(msg.wheeltick.flags, ros_msg.flags);
    ASSERT_EQ(msg.wheeltick.source, ros_msg.source);
    ASSERT_EQ(msg.wheeltick.ticks, ros_msg.ticks);
    ASSERT_EQ(msg.wheeltick.time, ros_msg.time);
  };

  testPublisher<swiftnav_ros2_driver::msg::Wheeltick>(
      Publishers::Wheeltick, msg, SbpMsgWheeltick, check);
}
