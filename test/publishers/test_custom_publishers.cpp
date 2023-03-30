/*
 * Copyright (C) 2015-2023 Swift Navigation Inc.
 * Contact: https://support.swiftnav.com
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

#include <utils/config.h>
#include <swiftnav_ros2_driver/msg/baseline.hpp>


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
  void testPublisher(const std::string& pub_type, const sbpT& msg,
                     const sbp_msg_type_t msg_type, Func comp) {
    bool test_finished = false;
    bool timed_out = false;
    auto node = std::make_shared<rclcpp::Node>("TestCustomPublishersNode");
    auto node_ptr = node.get();
    auto config = std::make_shared<Config>(node_ptr);
    auto pub = publisherFactory(pub_type, runner_.getState(), node.get(),
                                logger_, frame_name_, config);
    auto subs_call = [&msg, &test_finished, &comp](const rosT ros_msg) {
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
  auto node_ptr = node.get();
  auto config = std::make_shared<Config>(node_ptr);
  auto pub = publisherFactory("invalid_one", runner_.getState(), node.get(),
                              logger_, frame_name_, config);
  ASSERT_FALSE(pub);
}

TEST_F(TestCustomPublishers, CreateBaselinePublisher) {
}
