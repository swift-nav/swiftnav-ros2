#include <gtest/gtest.h>

#include <rclcpp/exceptions.hpp>
#include <rclcpp/rclcpp.hpp>

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
};

void wait_for_message_to_be_received(
  bool & is_received,
  const std::shared_ptr<rclcpp::Node> & node)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_once(std::chrono::milliseconds(0));
  int i = 0;
  while (!is_received && i < g_max_loops) {
    printf("spin_node_once() - callback (1) expected - try %d/%d\n", ++i, g_max_loops);
    executor.spin_once(g_sleep_per_loop);
  }
}

bool wait_for_match(
  const std::shared_ptr<rclcpp::SubscriptionBase> sub,
  const std::shared_ptr<rclcpp::PublisherBase> pub)
{
  int i = 0;
  bool matched = false;
  while (!matched && i < g_max_loops) {
    matched = sub->get_publisher_count() > 0 && pub->get_subscription_count() > 0;
    std::this_thread::sleep_for(g_sleep_per_loop);
  }
  return matched;
}

TEST_F(TestNavSatFixPublisher, sendMessage) {
    

}
