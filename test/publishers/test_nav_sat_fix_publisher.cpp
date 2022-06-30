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

#include <rclcpp/exceptions.hpp>
#include <rclcpp/rclcpp.hpp>

#include<test/test_utils.h>
#include<test/mocked_logger.h>

#include <publishers/navsatfix_publisher.h>
#include <utils/config.h>

class TestNavSatFixPublisher : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

 const std::string topic_name_ = "test_nav_sat_fix";
 const std::string frame_name_ = "test_frame";
 sbp::State state_;

};

TEST_F(TestNavSatFixPublisher, sendMessage) {

 auto node = std::make_shared<rclcpp::Node>("TestNavSatFixNode");
 auto ml = std::make_shared<MockedLogger>();
 auto node_ptr = node.get();
 auto config = std::make_shared<Config>(node_ptr);
 NavSatFixPublisher nav_sat_fix_publisher(&state_, topic_name_, node.get(), ml,
                                          frame_name_, config);

 sbp_msg_t obs_sbp_msg;
 obs_sbp_msg.obs.header.t.tow = 1;
 obs_sbp_msg.obs.n_obs = 1;
 sbp_packed_obs_content_t obs_content;
 obs_content.sid.code = 0;
 obs_sbp_msg.obs.obs[0] = obs_content;

 sbp_msg_t pos_llh_cov_sbp_msg;
 pos_llh_cov_sbp_msg.pos_llh_cov.tow = 2;
 pos_llh_cov_sbp_msg.pos_llh_cov.lat = 3;
 pos_llh_cov_sbp_msg.pos_llh_cov.lon = 4;
 pos_llh_cov_sbp_msg.pos_llh_cov.height = 10;

 pos_llh_cov_sbp_msg.pos_llh_cov.cov_e_e = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_e = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_e_d = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_e = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_n = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_d = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_e_d = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_d = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_d_d = 1;

 pos_llh_cov_sbp_msg.pos_llh_cov.flags = 1;

 bool is_received = false;
 auto callback =
  [&is_received](
    const sensor_msgs::msg::NavSatFix::SharedPtr msg) -> void {
      is_received = true;

      ASSERT_EQ(msg->status.service, sensor_msgs::msg::NavSatStatus::SERVICE_GPS);

      ASSERT_EQ(msg->latitude, 3);
      ASSERT_EQ(msg->longitude, 4);
      ASSERT_EQ(msg->altitude, 10);

      ASSERT_EQ(msg->position_covariance[0], 1);
      ASSERT_EQ(msg->position_covariance[1], 1);
      ASSERT_EQ(msg->position_covariance[2], -1);
      ASSERT_EQ(msg->position_covariance[3], 1);
      ASSERT_EQ(msg->position_covariance[4], 1);
      ASSERT_EQ(msg->position_covariance[5], -1);
      ASSERT_EQ(msg->position_covariance[6], -1);
      ASSERT_EQ(msg->position_covariance[7], -1);
      ASSERT_EQ(msg->position_covariance[8], 1);

      ASSERT_EQ(msg->position_covariance_type, sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN);
      ASSERT_EQ(msg->status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);

  };
 auto sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(topic_name_, 1, callback);
 nav_sat_fix_publisher.handle_sbp_msg(0, pos_llh_cov_sbp_msg.pos_llh_cov);
 ASSERT_FALSE(is_received);
 wait_for_message_to_be_received(is_received, node);
 ASSERT_TRUE(is_received);

}

TEST_F(TestNavSatFixPublisher, SERVICE_GPS_StatusService) {

 auto node = std::make_shared<rclcpp::Node>("TestNavSatFixNode");
 auto ml = std::make_shared<MockedLogger>();
 auto node_ptr = node.get();
 auto config = std::make_shared<Config>(node_ptr);
 NavSatFixPublisher nav_sat_fix_publisher(&state_, topic_name_, node.get(), ml,
                                          frame_name_, config);

 sbp_msg_t obs_sbp_msg;
 obs_sbp_msg.obs.header.t.tow = 1;
 obs_sbp_msg.obs.n_obs = 1;
 sbp_packed_obs_content_t obs_content;
 obs_content.sid.code = 0;
 obs_sbp_msg.obs.obs[0] = obs_content;

 sbp_msg_t pos_llh_cov_sbp_msg;
 pos_llh_cov_sbp_msg.pos_llh_cov.tow = 2;
 pos_llh_cov_sbp_msg.pos_llh_cov.lat = 3;
 pos_llh_cov_sbp_msg.pos_llh_cov.lon = 4;
 pos_llh_cov_sbp_msg.pos_llh_cov.height = 10;

 pos_llh_cov_sbp_msg.pos_llh_cov.cov_e_e = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_e = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_e_d = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_e = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_n = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_d = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_e_d = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_d = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_d_d = 1;

 pos_llh_cov_sbp_msg.pos_llh_cov.flags = 1;

 bool is_received = false;
 auto callback =
  [&is_received](
    const sensor_msgs::msg::NavSatFix::SharedPtr msg) -> void {
      is_received = true;

      ASSERT_EQ(msg->status.service, sensor_msgs::msg::NavSatStatus::SERVICE_GPS);

      ASSERT_EQ(msg->latitude, 3);
      ASSERT_EQ(msg->longitude, 4);
      ASSERT_EQ(msg->altitude, 10);

      ASSERT_EQ(msg->position_covariance[0], 1);
      ASSERT_EQ(msg->position_covariance[1], 1);
      ASSERT_EQ(msg->position_covariance[2], -1);
      ASSERT_EQ(msg->position_covariance[3], 1);
      ASSERT_EQ(msg->position_covariance[4], 1);
      ASSERT_EQ(msg->position_covariance[5], -1);
      ASSERT_EQ(msg->position_covariance[6], -1);
      ASSERT_EQ(msg->position_covariance[7], -1);
      ASSERT_EQ(msg->position_covariance[8], 1);

      ASSERT_EQ(msg->position_covariance_type, sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN);
      ASSERT_EQ(msg->status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);

  };
 auto sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(topic_name_, 1, callback);

 u8 values[] = {0, 1, 5, 7, 8, 9, 10, 11, 56, 57, 58, 2, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43};
 for(u8 &val : values) {
   is_received = false;
   obs_content.sid.code = val;
   obs_sbp_msg.obs.obs[0] = obs_content;
   nav_sat_fix_publisher.handle_sbp_msg(0, pos_llh_cov_sbp_msg.pos_llh_cov);
   ASSERT_FALSE(is_received);
   wait_for_message_to_be_received(is_received, node);
   ASSERT_TRUE(is_received);
 }

}

TEST_F(TestNavSatFixPublisher, SERVICE_GLONAS_StatusService) {

 auto node = std::make_shared<rclcpp::Node>("TestNavSatFixNode");
 auto ml = std::make_shared<MockedLogger>();
 auto node_ptr = node.get();
 auto config = std::make_shared<Config>(node_ptr);
 NavSatFixPublisher nav_sat_fix_publisher(&state_, topic_name_, node.get(), ml,
                                          frame_name_, config);

 sbp_msg_t obs_sbp_msg;
 obs_sbp_msg.obs.header.t.tow = 1;
 obs_sbp_msg.obs.n_obs = 1;
 sbp_packed_obs_content_t obs_content;
 obs_content.sid.code = 0;
 obs_sbp_msg.obs.obs[0] = obs_content;

 sbp_msg_t pos_llh_cov_sbp_msg;
 pos_llh_cov_sbp_msg.pos_llh_cov.tow = 2;
 pos_llh_cov_sbp_msg.pos_llh_cov.lat = 3;
 pos_llh_cov_sbp_msg.pos_llh_cov.lon = 4;
 pos_llh_cov_sbp_msg.pos_llh_cov.height = 10;

 pos_llh_cov_sbp_msg.pos_llh_cov.cov_e_e = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_e = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_e_d = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_e = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_n = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_d = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_e_d = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_d = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_d_d = 1;

 pos_llh_cov_sbp_msg.pos_llh_cov.flags = 1;

 bool is_received = false;
 auto callback =
  [&is_received](
    const sensor_msgs::msg::NavSatFix::SharedPtr msg) -> void {
      is_received = true;

      ASSERT_EQ(msg->status.service, sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS);

      ASSERT_EQ(msg->latitude, 3);
      ASSERT_EQ(msg->longitude, 4);
      ASSERT_EQ(msg->altitude, 10);

      ASSERT_EQ(msg->position_covariance[0], 1);
      ASSERT_EQ(msg->position_covariance[1], 1);
      ASSERT_EQ(msg->position_covariance[2], -1);
      ASSERT_EQ(msg->position_covariance[3], 1);
      ASSERT_EQ(msg->position_covariance[4], 1);
      ASSERT_EQ(msg->position_covariance[5], -1);
      ASSERT_EQ(msg->position_covariance[6], -1);
      ASSERT_EQ(msg->position_covariance[7], -1);
      ASSERT_EQ(msg->position_covariance[8], 1);

      ASSERT_EQ(msg->position_covariance_type, sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN);
      ASSERT_EQ(msg->status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);

  };
 auto sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(topic_name_, 1, callback);

 u8 values[] = {3, 4, 29, 30};
 for(u8 &val : values) {
   is_received = false;
   obs_content.sid.code = val;
   obs_sbp_msg.obs.obs[0] = obs_content;
   nav_sat_fix_publisher.handle_sbp_msg(0, pos_llh_cov_sbp_msg.pos_llh_cov);
   ASSERT_FALSE(is_received);
   wait_for_message_to_be_received(is_received, node);
   ASSERT_TRUE(is_received);
 }
}

TEST_F(TestNavSatFixPublisher, SERVICE_GALILEO_StatusService) {
  auto node = std::make_shared<rclcpp::Node>("TestNavSatFixNode");
  auto ml = std::make_shared<MockedLogger>();
  auto node_ptr = node.get();
  auto config = std::make_shared<Config>(node_ptr);
  NavSatFixPublisher nav_sat_fix_publisher(&state_, topic_name_, node.get(), ml,
                                           frame_name_, config);

  sbp_msg_t obs_sbp_msg;
  obs_sbp_msg.obs.header.t.tow = 1;
  obs_sbp_msg.obs.n_obs = 1;
  sbp_packed_obs_content_t obs_content;
  obs_content.sid.code = 0;
  obs_sbp_msg.obs.obs[0] = obs_content;

  sbp_msg_t pos_llh_cov_sbp_msg;
  pos_llh_cov_sbp_msg.pos_llh_cov.tow = 2;
  pos_llh_cov_sbp_msg.pos_llh_cov.lat = 3;
  pos_llh_cov_sbp_msg.pos_llh_cov.lon = 4;
  pos_llh_cov_sbp_msg.pos_llh_cov.height = 10;

  pos_llh_cov_sbp_msg.pos_llh_cov.cov_e_e = 1;
  pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_e = 1;
  pos_llh_cov_sbp_msg.pos_llh_cov.cov_e_d = 1;
  pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_e = 1;
  pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_n = 1;
  pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_d = 1;
  pos_llh_cov_sbp_msg.pos_llh_cov.cov_e_d = 1;
  pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_d = 1;
  pos_llh_cov_sbp_msg.pos_llh_cov.cov_d_d = 1;

  pos_llh_cov_sbp_msg.pos_llh_cov.flags = 1;

  bool is_received = false;
  auto callback =
      [&is_received](const sensor_msgs::msg::NavSatFix::SharedPtr msg) -> void {
    is_received = true;

    ASSERT_EQ(msg->status.service,
              sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO);

    ASSERT_EQ(msg->latitude, 3);
    ASSERT_EQ(msg->longitude, 4);
    ASSERT_EQ(msg->altitude, 10);

    ASSERT_EQ(msg->position_covariance[0], 1);
    ASSERT_EQ(msg->position_covariance[1], 1);
    ASSERT_EQ(msg->position_covariance[2], -1);
    ASSERT_EQ(msg->position_covariance[3], 1);
    ASSERT_EQ(msg->position_covariance[4], 1);
    ASSERT_EQ(msg->position_covariance[5], -1);
    ASSERT_EQ(msg->position_covariance[6], -1);
    ASSERT_EQ(msg->position_covariance[7], -1);
    ASSERT_EQ(msg->position_covariance[8], 1);

    ASSERT_EQ(msg->position_covariance_type,
              sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN);
    ASSERT_EQ(msg->status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);
  };
  auto sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(
      topic_name_, 1, callback);

  u8 values[] = {14, 28};
  for (u8 &val : values) {
    is_received = false;
    obs_content.sid.code = val;
    obs_sbp_msg.obs.obs[0] = obs_content;
    nav_sat_fix_publisher.handle_sbp_msg(0, pos_llh_cov_sbp_msg.pos_llh_cov);
    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
}

TEST_F(TestNavSatFixPublisher, SERVICE_COMPASS_StatusService) {

 auto node = std::make_shared<rclcpp::Node>("TestNavSatFixNode");
 auto ml = std::make_shared<MockedLogger>();
 auto node_ptr = node.get();
 auto config = std::make_shared<Config>(node_ptr);
 NavSatFixPublisher nav_sat_fix_publisher(&state_, topic_name_, node.get(), ml,
                                          frame_name_, config);

 sbp_msg_t obs_sbp_msg;
 obs_sbp_msg.obs.header.t.tow = 1;
 obs_sbp_msg.obs.n_obs = 1;
 sbp_packed_obs_content_t obs_content;
 obs_content.sid.code = 0;
 obs_sbp_msg.obs.obs[0] = obs_content;

 sbp_msg_t pos_llh_cov_sbp_msg;
 pos_llh_cov_sbp_msg.pos_llh_cov.tow = 2;
 pos_llh_cov_sbp_msg.pos_llh_cov.lat = 3;
 pos_llh_cov_sbp_msg.pos_llh_cov.lon = 4;
 pos_llh_cov_sbp_msg.pos_llh_cov.height = 10;

 pos_llh_cov_sbp_msg.pos_llh_cov.cov_e_e = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_e = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_e_d = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_e = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_n = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_d = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_e_d = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_d = 1;
 pos_llh_cov_sbp_msg.pos_llh_cov.cov_d_d = 1;

 pos_llh_cov_sbp_msg.pos_llh_cov.flags = 1;

 bool is_received = false;
 auto callback =
     [&is_received](const sensor_msgs::msg::NavSatFix::SharedPtr msg) -> void {
   is_received = true;

   ASSERT_EQ(msg->status.service,
             sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS);

   ASSERT_EQ(msg->latitude, 3);
   ASSERT_EQ(msg->longitude, 4);
   ASSERT_EQ(msg->altitude, 10);

   ASSERT_EQ(msg->position_covariance[0], 1);
   ASSERT_EQ(msg->position_covariance[1], 1);
   ASSERT_EQ(msg->position_covariance[2], -1);
   ASSERT_EQ(msg->position_covariance[3], 1);
   ASSERT_EQ(msg->position_covariance[4], 1);
   ASSERT_EQ(msg->position_covariance[5], -1);
   ASSERT_EQ(msg->position_covariance[6], -1);
   ASSERT_EQ(msg->position_covariance[7], -1);
   ASSERT_EQ(msg->position_covariance[8], 1);

   ASSERT_EQ(msg->position_covariance_type,
             sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN);
   ASSERT_EQ(msg->status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);
 };
 auto sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(topic_name_,
                                                                   1, callback);

 u8 values[] = {12, 13, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55};
 for (u8 &val : values) {
   is_received = false;
   obs_content.sid.code = val;
   obs_sbp_msg.obs.obs[0] = obs_content;
   nav_sat_fix_publisher.handle_sbp_msg(0, pos_llh_cov_sbp_msg.pos_llh_cov);
   ASSERT_FALSE(is_received);
   wait_for_message_to_be_received(is_received, node);
   ASSERT_TRUE(is_received);
  }
}

TEST_F(TestNavSatFixPublisher, timeDiff) {
  auto node = std::make_shared<rclcpp::Node>("TestNavSatFixNode");
  auto ml = std::make_shared<MockedLogger>();
 auto node_ptr = node.get();
 auto config = std::make_shared<Config>(node_ptr);
  NavSatFixPublisher nav_sat_fix_publisher(&state_, topic_name_, node.get(), ml,
                                           frame_name_, config);

  sbp_msg_t obs_sbp_msg;
  obs_sbp_msg.obs.header.t.tow = 1;
  obs_sbp_msg.obs.n_obs = 1;
  sbp_packed_obs_content_t obs_content;
  obs_content.sid.code = 0;
  obs_sbp_msg.obs.obs[0] = obs_content;

  sbp_msg_t pos_llh_cov_sbp_msg;
  pos_llh_cov_sbp_msg.pos_llh_cov.tow = 2002;
  pos_llh_cov_sbp_msg.pos_llh_cov.lat = 3;
  pos_llh_cov_sbp_msg.pos_llh_cov.lon = 4;
  pos_llh_cov_sbp_msg.pos_llh_cov.height = 10;

  pos_llh_cov_sbp_msg.pos_llh_cov.cov_e_e = 1;
  pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_e = 1;
  pos_llh_cov_sbp_msg.pos_llh_cov.cov_e_d = 1;
  pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_e = 1;
  pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_n = 1;
  pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_d = 1;
  pos_llh_cov_sbp_msg.pos_llh_cov.cov_e_d = 1;
  pos_llh_cov_sbp_msg.pos_llh_cov.cov_n_d = 1;
  pos_llh_cov_sbp_msg.pos_llh_cov.cov_d_d = 1;

  pos_llh_cov_sbp_msg.pos_llh_cov.flags = 1;

  bool is_received = false;
  auto callback =
      [&is_received](const sensor_msgs::msg::NavSatFix::SharedPtr /*msg*/) -> void {
    is_received = true;
  };
  auto sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(
      topic_name_, 1, callback);
  nav_sat_fix_publisher.handle_sbp_msg(0, pos_llh_cov_sbp_msg.pos_llh_cov);
  ASSERT_EQ(
      "Time difference between OBS message and POS_LLH_COV message is larger "
      "than Max",
      ml->getLastLoggedWarning());

  ASSERT_FALSE(is_received);
}
