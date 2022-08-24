#include<chrono>

#include <rclcpp/exceptions.hpp>
#include <rclcpp/rclcpp.hpp>

static const int g_max_loops = 200;
static const std::chrono::milliseconds g_sleep_per_loop(10);

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
