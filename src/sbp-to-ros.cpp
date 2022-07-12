#include <chrono>
#include <iostream>
#include <fstream>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <libsbp/cpp/state.h>
#include <libsbp/legacy/cpp/payload_handler.h>
#include <libsbp/legacy/cpp/frame_handler.h>


class SbpFileReader : public sbp::IReader {
 public:
  SbpFileReader(const char *file_path) : file_stream_(file_path, std::ios::binary | std::ios_base::in) { }

  bool is_open() const { return file_stream_.is_open(); }
  bool eof() const { return file_stream_.eof(); }

  s32 read(u8 *buffer, u32 buffer_length) override {
    auto start_index = file_stream_.tellg();
    if (start_index == -1) {
      return -1;
    }
    file_stream_.read(reinterpret_cast<char *>(buffer), buffer_length);
    auto end_index = file_stream_.tellg();
    if (end_index == -1 || file_stream_.fail()) {
      return -1;
    }

    return static_cast<s32>(end_index - start_index);
  }

 private:
  std::ifstream file_stream_;
};

class NavSatFixPublisher : private sbp::PayloadHandler<msg_gps_time_t, msg_pos_ecef_t> {
  public:
    ECEFHandler(std::shared_ptr<sbp::State> state, rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher) 
      : sbp::PayloadHandler<msg_gps_time_t, msg_pos_ecef_t>(*state), publisher_(publisher) {
       
    }

    void handle_sbp_msg(uint16_t sender_id, uint8_t message_length, const msg_gps_time_t& msg) {
      (void)sender_id;
      (void)message_length;
      std::cout << "Received new GPS_TME message with WN = " << msg.wn << ", TOW = " << msg.tow << "\n";
    }

    void handle_sbp_msg(uint16_t sender_id, uint8_t message_length, const msg_pos_ecef_t& msg) {
      (void)sender_id;
      (void)message_length;
      std::cout << "Received new POS_ECEF message with TOW = " << msg.tow;
      std::cout << ", (X,Y,Z) = (" << msg.x << "," << msg.y << "," << msg.z << ")\n";
      auto message = sensor_msgs::msg::NavSatFix();
      publisher_->publish(message);
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
};

class SBPROS2DriverNode : public rclcpp::Node
{
  public:
    SBPROS2DriverNode()
    : Node("SBPRRos2Driver")
    {
      /* Parameters */
      this->declare_parameter<std::string>("sbp_file");
      this->get_parameter<std::string>("sbp_file", sbp_file_);

      /* Publisher */
      publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("topic", 10);

      SbpFileReader reader(sbp_file_);
      if (!reader.is_open()) {
        exit(EXIT_FAILURE);
      }

      NavSatFixPublisher navsat_fix_publisher(state_, publisher_);

     /* SBP Callbacks */

    }

  private:
    std::shared_ptr<sbp::State> state_;
    std::string sbp_file_;

};

int main(int argc, char ** argv)
{
  
    printf("hello world swiftnav-ros2 package\n");

  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("sbptoros2");

  sbp::State s;
  ECEFHandler ecef_handler(&s, node);
  LLHFrameHandler llh_handler(&s);
 // EverythingHandler everything_handler(&s);

  s.set_reader(&reader);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while(!reader.eof()) {
    s.process(); 
  }
 
  rclcpp::shutdown();

  return 0;
}
