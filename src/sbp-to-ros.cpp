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
    NavSatFixPublisher(sbp::State *state, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> publisher) 
      : sbp::PayloadHandler<msg_gps_time_t, msg_pos_ecef_t>(state), publisher_(publisher) {
       
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
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> publisher_;
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

      std::cout << "SBP_FILE --: " << sbp_file_ << std::endl;

      /* Publisher */
      auto publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("topic", 10);

      std::cout << "Publisher" << std::endl;

     reader_ = new SbpFileReader(sbp_file_.c_str());
      if (!reader_->is_open()) {
        exit(EXIT_FAILURE);
      }
      std::cout << "Reader" << std::endl;
      state_.set_reader(reader_);

      std::cout << "fixpublisher" << std::endl;
      navsatfixpublisher_ = new NavSatFixPublisher(&state_, publisher_);

     /* SBP Callbacks */
      sbp_thread_ = std::thread(&SBPROS2DriverNode::processSBP, this);

      std::cout << "start" << std::endl;

    }

    ~SBPROS2DriverNode(){
        exit_requested_.store(true);
        if (sbp_thread_.joinable()) sbp_thread_.join();
        delete(reader_);
        delete(navsatfixpublisher_);
    }

    void processSBP() {
      while(!exit_requested_.load() && !reader_->eof()) {
       state_.process(); 
      }  
    }

  private:
    sbp::State state_;
    std::string sbp_file_;
    std::thread sbp_thread_;
    std::atomic_bool exit_requested_ = false;
    SbpFileReader* reader_;
    NavSatFixPublisher* navsatfixpublisher_;

};

int main(int argc, char ** argv)
{
  
  printf("hello world swiftnav-ros2 package\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SBPROS2DriverNode>());
  rclcpp::shutdown();

  return 0;
}
