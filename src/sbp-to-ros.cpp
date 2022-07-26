#include <chrono>
#include <functional>
#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"

//#include <logging/ros_logger.h>

#include <libsbp/cpp/state.h>
#include <libsbp/cpp/message_handler.h>

#include <publishers/NavSatFixPublisher.h>
#include <publishers/TimeReferencePublisher.h>

#include <readers/sbp_filereader.h>
#include <readers/sbp_tcpreader.h>

// *************************************************************************
// Dummy console implementation of a Logger
class MockedLogger : public IIssueLogger {
 public:
  void logDebug(const std::stringstream& ss) override {
    std::cout << "DEBUG->" << ss.str() << std::endl;
  }
  void logInfo(const std::stringstream& ss) override {
    std::cout << "INFO->" << ss.str() << std::endl;
  }
  void logWarning(const std::stringstream& ss) override {
    std::cout << "WARN->" << ss.str() << std::endl;
  }
  void logError(const std::stringstream& ss) override {
    std::cout << "ERROR->" << ss.str() << std::endl;
  }
  void logFatal(const std::stringstream& ss) override {
    std::cout << "FATAL->" << ss.str() << std::endl;
  }
};

// Look at this as an alternative way
// class EverythingHandler : private sbp::AllMessageHandler {
//  public:
//   EverythingHandler(sbp::State* state) : sbp::AllMessageHandler(state) {}

//   void handle_sbp_message(uint16_t sender_id, sbp_msg_type_t msg_type,
//                           const sbp_msg_t& msg) {
//     (void)sender_id;
//     (void)msg;
//     std::cout << "Received new message, message type " << msg_type << "\n";
//   }
// };

// everything_handler = std::make_shared<EverythingHandler>(&state_);
// std::shared_ptr<EverythingHandler> everything_handler;

class SBPROS2DriverNode : public rclcpp::Node {
 public:
  SBPROS2DriverNode() : Node("SBPRRos2Driver") {
    /* Declare Parameters */
    declare_parameter<std::string>("host_ip", "");
    declare_parameter<uint16_t>("host_port", 0);
    declare_parameter<uint8_t>("interface", 0);
    declare_parameter<std::string>("sbp_file", "");
    declare_parameter<bool>("publish_navsatfix", true);
    declare_parameter<bool>("publish_timereference", true);

    logger_ = std::make_shared<MockedLogger>();

    /* Get Parameters */
    uint8_t interface;
    get_parameter<uint8_t>("interface", interface);
    if (interface == 0) {
      std::string ip;
      uint16_t port;

      get_parameter<std::string>("host_ip", ip);
      get_parameter<uint16_t>("host_port", port);
      tcp_reader_ = std::make_unique<SBPTCPReader>(ip, port, logger_, 2000);
      if (!tcp_reader_->isValid()) exit(EXIT_FAILURE);
      std::cout << "Using TCP Reader\n";
    } else {
      get_parameter<std::string>("sbp_file", sbp_file_);
      std::cout << "SBP_FILE --: " << sbp_file_ << std::endl;
      reader_ptr_ = std::make_unique<SbpFileReader>(sbp_file_);
      if (!reader_ptr_->is_open()) exit(EXIT_FAILURE);
      std::cout << "Using File Reader\n";
    }

    get_parameter<bool>("publish_navsatfix", publish_navsatfix_);
    get_parameter<bool>("publish_timereference", publish_timereference_);

    if (tcp_reader_)
      state_.set_reader(tcp_reader_.get());
    else
      state_.set_reader(reader_ptr_.get());

    /* Publishers */
    if (publish_navsatfix_) {
      auto navsatfix_publisher_ =
          create_publisher<sensor_msgs::msg::NavSatFix>("navsatfix", 10);
      navsatfix_publisher_ptr_ = std::make_unique<NavSatFixPublisher>(
          &state_, navsatfix_publisher_, this);
    }
    if (publish_timereference_) {
      auto timereference_publisher_ =
          create_publisher<sensor_msgs::msg::TimeReference>("timereference",
                                                            10);
      timereference_publisher_ptr_ = std::make_unique<TimeReferencePublisher>(
          &state_, timereference_publisher_, this);
    }

    /* SBP Callback processing thread */
    if (tcp_reader_)
      sbp_thread_ = std::thread(&SBPROS2DriverNode::processSBPNet, this);
    else
      sbp_thread_ = std::thread(&SBPROS2DriverNode::processSBPFile, this);

    std::cout << "start" << std::endl;
  }

  ~SBPROS2DriverNode() {
    exit_requested_.store(true);
    if (sbp_thread_.joinable()) sbp_thread_.join();
  }

  void processSBPFile() {
    while (!exit_requested_.load() && !reader_ptr_->eof()) {
      state_.process();
    }
  }

  void processSBPNet() {
    while (!exit_requested_.load()) {
      state_.process();
    }
  }

 private:
  sbp::State state_;
  std::thread sbp_thread_;
  std::atomic_bool exit_requested_ = false;
  std::unique_ptr<SbpFileReader> reader_ptr_;
  std::unique_ptr<SBPTCPReader> tcp_reader_;
  std::shared_ptr<MockedLogger> logger_;

  std::string sbp_file_;
  bool publish_navsatfix_{true};
  bool publish_timereference_{true};

  std::unique_ptr<NavSatFixPublisher> navsatfix_publisher_ptr_;
  std::unique_ptr<TimeReferencePublisher> timereference_publisher_ptr_;
};

int main(int argc, char** argv) {
  printf("hello world swiftnav-ros2 package\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SBPROS2DriverNode>());
  rclcpp::shutdown();

  return 0;
}
