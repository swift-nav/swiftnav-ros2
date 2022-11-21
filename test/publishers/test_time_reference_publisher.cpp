#include <gtest/gtest.h>

#include<test/test_utils.h>
#include<test/mocked_logger.h>

#include <rclcpp/exceptions.hpp>
#include <rclcpp/rclcpp.hpp>

#include <libsbp/cpp/state.h>

#include <publishers/timereference_publisher.h>

class TestTimeReferencePublisher : public ::testing::Test
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

 const std::string topic_name_ = "test_time_reference";
 const std::string frame_name_ = "test_frame";
 sbp::State state_;

};

TEST_F(TestTimeReferencePublisher, sendMessage) {
 auto node = std::make_shared<rclcpp::Node>("TestTimeReferenceNode");
 auto ml = std::make_shared<MockedLogger>();
 TimeReferencePublisher time_reference_publisher(&state_, topic_name_,
                                                 node.get(), ml, frame_name_);

 sbp_msg_t sbp_msg;
 sbp_msg.gps_time.tow = 1000;
 sbp_msg.gps_time.ns_residual = 2;

 bool is_received = false;
 auto callback =
  [&is_received](
    const sensor_msgs::msg::TimeReference & msg) -> void {
      is_received = true;

      ASSERT_EQ(msg.time_ref.sec, 1);
      ASSERT_FLOAT_EQ(msg.time_ref.nanosec, 2);
    };
 auto sub = node->create_subscription<sensor_msgs::msg::TimeReference>(topic_name_, 1, callback);
 time_reference_publisher.handle_sbp_msg(0, sbp_msg.gps_time);
 ASSERT_FALSE(is_received);
 wait_for_message_to_be_received(is_received, node);
 ASSERT_TRUE(is_received);

}
