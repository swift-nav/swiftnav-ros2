#include <gtest/gtest.h>

#include <rclcpp/exceptions.hpp>
#include <rclcpp/rclcpp.hpp>

#include<test/test_utils.h>
#include<test/mocked_logger.h>

#include <publishers/NavSatFixPublisher.h>

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
 NavSatFixPublisher nav_sat_fix_publisher(&state_, topic_name_, node.get(), ml, true, frame_name_);

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
    const sensor_msgs::msg::NavSatFix & msg) -> void {
      is_received = true;

      ASSERT_EQ(msg.status.service, sensor_msgs::msg::NavSatStatus::SERVICE_GPS);
      ASSERT_EQ(msg.status.service, sensor_msgs::msg::NavSatStatus::SERVICE_GPS);

      ASSERT_EQ(msg.latitude, 3);
      ASSERT_EQ(msg.longitude, 4);
      ASSERT_EQ(msg.altitude, 10);

      ASSERT_EQ(msg.position_covariance[0], 1);
      ASSERT_EQ(msg.position_covariance[1], 1);
      ASSERT_EQ(msg.position_covariance[2], -1);
      ASSERT_EQ(msg.position_covariance[3], 1);
      ASSERT_EQ(msg.position_covariance[4], 1);
      ASSERT_EQ(msg.position_covariance[5], -1);
      ASSERT_EQ(msg.position_covariance[6], -1);
      ASSERT_EQ(msg.position_covariance[7], -1);
      ASSERT_EQ(msg.position_covariance[8], 1);

      ASSERT_EQ(msg.position_covariance_type, sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN);
      ASSERT_EQ(msg.status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);

  };
 auto sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(topic_name_, 1, callback);
 nav_sat_fix_publisher.handle_sbp_msg(0, obs_sbp_msg.obs);
 nav_sat_fix_publisher.handle_sbp_msg(0, pos_llh_cov_sbp_msg.pos_llh_cov);
 ASSERT_FALSE(is_received);
 wait_for_message_to_be_received(is_received, node);
 ASSERT_TRUE(is_received);

}

TEST_F(TestNavSatFixPublisher, timeDiff) {
 
 auto node = std::make_shared<rclcpp::Node>("TestNavSatFixNode");
 auto ml = std::make_shared<MockedLogger>();
 NavSatFixPublisher nav_sat_fix_publisher(&state_, topic_name_, node.get(), ml, true, frame_name_);

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
  [&is_received](
    const sensor_msgs::msg::NavSatFix & msg) -> void {
      is_received = true;
  };
 auto sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(topic_name_, 1, callback);
 nav_sat_fix_publisher.handle_sbp_msg(0, obs_sbp_msg.obs);
 nav_sat_fix_publisher.handle_sbp_msg(0, pos_llh_cov_sbp_msg.pos_llh_cov);
 ASSERT_EQ("Time difference between OBS message and POS_LLH_COV message is larger than Max", ml->getLastLoggedWarning());
 
 ASSERT_FALSE(is_received);


}