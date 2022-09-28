#include <gtest/gtest.h>

#include <rclcpp/exceptions.hpp>
#include <rclcpp/rclcpp.hpp>

#include<test/test_utils.h>
#include<test/mocked_logger.h>

#include <publishers/publisher_factory.h>

class TestGPSFixPublisher : public ::testing::Test
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

 const std::string topic_name_ = "test_gps_fix";
 const std::string frame_name_ = "test_frame";
 sbp::State state_;

};

TEST_F(TestGPSFixPublisher, sendMessage) {
  auto node = std::make_shared<rclcpp::Node>("TestGPSFixNode");
  auto ml = std::make_shared<MockedLogger>();
  auto gps_fix_publisher = publisherFactory(
      Publishers::GpsFix, &state_, topic_name_, node.get(), ml, frame_name_);
  ASSERT_TRUE(gps_fix_publisher);

  sbp_msg_t pos_llh_acc_sbp_msg;
  pos_llh_acc_sbp_msg.pos_llh_acc.tow = 1;
  pos_llh_acc_sbp_msg.pos_llh_acc.n_sats = 3;
  pos_llh_acc_sbp_msg.pos_llh_acc.h_accuracy = 0.1;
  pos_llh_acc_sbp_msg.pos_llh_acc.v_accuracy = 0.2;
  pos_llh_acc_sbp_msg.pos_llh_acc.at_accuracy = 0.3;

  sbp_msg_t pos_llh_cov_sbp_msg;
  pos_llh_cov_sbp_msg.pos_llh_cov.tow = 1;
  pos_llh_cov_sbp_msg.pos_llh_cov.lat = 10;
  pos_llh_cov_sbp_msg.pos_llh_cov.lon = 20;
  pos_llh_cov_sbp_msg.pos_llh_cov.height = 5;

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

  sbp_msg_t vel_cog_sbp_msg;
  vel_cog_sbp_msg.vel_cog.tow = 1;
  vel_cog_sbp_msg.vel_cog.cog = 2;
  vel_cog_sbp_msg.vel_cog.sog = 2;
  vel_cog_sbp_msg.vel_cog.v_up = 2;
  vel_cog_sbp_msg.vel_cog.sog_accuracy = 2;
  vel_cog_sbp_msg.vel_cog.v_up_accuracy = 2;

  sbp_msg_t vel_ned_cov_sbp_msg;
  vel_ned_cov_sbp_msg.vel_ned_cov.tow = 1;

  sbp_msg_t orient_euler_sbp_msg;
  orient_euler_sbp_msg.orient_euler.tow = 1;
  orient_euler_sbp_msg.orient_euler.pitch = 2;
  orient_euler_sbp_msg.orient_euler.roll = 2;
  orient_euler_sbp_msg.orient_euler.yaw = 2;
  orient_euler_sbp_msg.orient_euler.pitch_accuracy = 2;
  orient_euler_sbp_msg.orient_euler.roll_accuracy = 2;
  orient_euler_sbp_msg.orient_euler.yaw_accuracy = 2;

  sbp_msg_t dops_sbp_msg;
  dops_sbp_msg.dops.tow = 1;
  dops_sbp_msg.dops.gdop = 2;
  dops_sbp_msg.dops.pdop = 3;
  dops_sbp_msg.dops.hdop = 4;
  dops_sbp_msg.dops.vdop = 5;
  dops_sbp_msg.dops.tdop = 6;

  sbp_msg_t gps_time_sbp_msg;
  gps_time_sbp_msg.gps_time.tow = 1;

  sbp_msg_t obs_sbp_msg;
  obs_sbp_msg.obs.header.t.tow = 1;
  obs_sbp_msg.obs.n_obs = 1;
  sbp_packed_obs_content_t obs_content;
  obs_content.sid.code = 10;
  obs_sbp_msg.obs.obs[0] = obs_content;

  bool is_received = false;
  auto callback = [&is_received](const gps_msgs::msg::GPSFix& msg) -> void {
    is_received = true;

    //      ASSERT_EQ(msg.status.service,
    //      sensor_msgs::msg::NavSatStatus::SERVICE_GPS);

    ASSERT_EQ(msg.status.satellites_used, 3);
    ASSERT_EQ(msg.err_horz, 3);
    ASSERT_EQ(msg.err_vert, 3);
    ASSERT_EQ(msg.err_track, 3);

    ASSERT_EQ(msg.latitude, 3);
    ASSERT_EQ(msg.longitude, 4);
    ASSERT_EQ(msg.altitude, 10);

    ASSERT_EQ(msg.track, 3);
    ASSERT_EQ(msg.speed, 4);
    ASSERT_EQ(msg.climb, 10);
    ASSERT_EQ(msg.err_speed, 10);
    ASSERT_EQ(msg.err_climb, 10);

    ASSERT_EQ(msg.pitch, 3);
    ASSERT_EQ(msg.roll, 4);
    ASSERT_EQ(msg.dip, 10);
    ASSERT_EQ(msg.err_pitch, 10);
    ASSERT_EQ(msg.err_roll, 10);
    ASSERT_EQ(msg.err_dip, 10);

    ASSERT_EQ(msg.gdop, 3);
    ASSERT_EQ(msg.pdop, 4);
    ASSERT_EQ(msg.hdop, 10);
    ASSERT_EQ(msg.vdop, 10);
    ASSERT_EQ(msg.tdop, 10);

    ASSERT_EQ(msg.position_covariance[0], 1);
    ASSERT_EQ(msg.position_covariance[1], 1);
    ASSERT_EQ(msg.position_covariance[2], -1);
    ASSERT_EQ(msg.position_covariance[3], 1);
    ASSERT_EQ(msg.position_covariance[4], 1);
    ASSERT_EQ(msg.position_covariance[5], -1);
    ASSERT_EQ(msg.position_covariance[6], -1);
    ASSERT_EQ(msg.position_covariance[7], -1);
    ASSERT_EQ(msg.position_covariance[8], 1);

    ASSERT_EQ(msg.status.satellites_visible, 1);
  };
  auto sub = node->create_subscription<gps_msgs::msg::GPSFix>(topic_name_, 1,
                                                              callback);
  nav_sat_fix_publisher.handle_sbp_msg(0, obs_sbp_msg.obs);
  nav_sat_fix_publisher.handle_sbp_msg(0, pos_llh_cov_sbp_msg.pos_llh_cov);
  ASSERT_FALSE(is_received);
  wait_for_message_to_be_received(is_received, node);
  ASSERT_TRUE(is_received);
}