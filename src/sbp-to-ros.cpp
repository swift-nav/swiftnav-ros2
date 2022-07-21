#include <chrono>
#include <iostream>
#include <fstream>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <libsbp/cpp/state.h>
#include <libsbp/cpp/message_handler.h>

#include <publishers/NavSatFixPublisher.h>
#include <publishers/TimeReferencePublisher.h>

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

class SBPROS2DriverNode : public rclcpp::Node
{
  public:
    SBPROS2DriverNode()
    : Node("SBPRRos2Driver")
    {
      /* Declare Parameters */
      this->declare_parameter<std::string>("sbp_file");
      this->declare_parameter<bool>("publish_navsatfix", true);
      this->declare_parameter<bool>("publish_timereference", true);

      /* Get Parameters */
      this->get_parameter<std::string>("sbp_file", sbp_file_);
      this->get_parameter<bool>("publish_navsatfix", publish_navsatfix_);
      this->get_parameter<bool>("publish_timereference", publish_timereference_);

      std::cout << "SBP_FILE --: " << sbp_file_ << std::endl;
     reader_ptr_ = std::move(std::make_unique<SbpFileReader>(sbp_file_.c_str()));
      if (!reader_ptr_->is_open()) {
        exit(EXIT_FAILURE);
      }
      std::cout << "Reader" << std::endl;
      state_.set_reader(reader_ptr_.get());

     /* Publishers */
      if (publish_navsatfix_) {
        auto navsatfix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("navsatfix", 10);
        navsatfix_publisher_ptr_ = std::move(std::make_unique<NavSatFixPublisher>(&state_, navsatfix_publisher_, this));
      }      
      if (publish_timereference_) {
        auto timereference_publisher_ = this->create_publisher<sensor_msgs::msg::TimeReference>("timereference", 10);
        timereference_publisher_ptr_ = std::move(std::make_unique<TimeReferencePublisher>(&state_, timereference_publisher_, this));
      }      

            
     /* SBP Callback processing thread */
      sbp_thread_ = std::thread(&SBPROS2DriverNode::processSBP, this);

      std::cout << "start" << std::endl;

    }

    ~SBPROS2DriverNode(){
        exit_requested_.store(true);
        if (sbp_thread_.joinable()) sbp_thread_.join();
    }

    void processSBP() {
      while(!exit_requested_.load() && !reader_ptr_->eof()) {
       state_.process(); 
      }  
    }

  private:
    sbp::State state_;
    std::thread sbp_thread_;
    std::atomic_bool exit_requested_ = false;
    std::unique_ptr<SbpFileReader> reader_ptr_;

    std::string sbp_file_;
    bool publish_navsatfix_{true};
    bool publish_timereference_{true};

    std::unique_ptr<NavSatFixPublisher> navsatfix_publisher_ptr_;
    std::unique_ptr<TimeReferencePublisher> timereference_publisher_ptr_;

};

int main(int argc, char ** argv)
{
  
  printf("hello world swiftnav-ros2 package\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SBPROS2DriverNode>());
  rclcpp::shutdown();

  return 0;
}
