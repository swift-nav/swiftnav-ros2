#include <chrono>
#include <functional>
#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <logging/ros_logger.h>

#include <libsbp/cpp/state.h>
#include <libsbp/cpp/message_handler.h>

#include <publishers/NavSatFixPublisher.h>
#include <publishers/TimeReferencePublisher.h>

#include <data_sources/sbp_data_sources.h>

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

/**
 * @brief Class that represents the ROS 2 driver node
 */
class SBPROS2DriverNode : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new SBPROS2DriverNode object
   */
  SBPROS2DriverNode() : Node("SBPRRos2Driver") {
    declareParameters();
    logger_ = std::make_shared<ROSLogger>();
    createReader();
    if (!reader_) exit(EXIT_FAILURE);
    state_.set_reader(reader_.get());
    createPublishers();

    /* SBP Callback processing thread */
    sbp_thread_ = std::thread(&SBPROS2DriverNode::processSBP, this);

    std::cout << "start" << std::endl;
  }

  /**
   * @brief Destroy the SBPROS2DriverNode object
   */
  ~SBPROS2DriverNode() {
    exit_requested_ = true;
    if (sbp_thread_.joinable()) sbp_thread_.join();
  }

  /**
   * @brief SBP messages processing thread
   */
  void processSBP() {
    while (!exit_requested_) {
      state_.process();
    }
  }

 private:
  /**
   * @brief Method for creating the readers (data sources)
   */
  void createReader() {
    uint8_t interface;

    get_parameter<uint8_t>("interface", interface);
    switch (interface) {
      case FILE_DATA_SOURCE: {
        std::string file;
        get_parameter<std::string>("sbp_file", file);
        reader_ = dataSourceFactory(file);
      } break;

      case SERIAL_DATA_SOURCE: {
        std::string device, connection_str;
        uint32_t timeout;
        get_parameter<std::string>("device_name", device);
        get_parameter<std::string>("connection_str", connection_str);
        get_parameter<uint32_t>("timeout", timeout);
        reader_ = dataSourceFactory(device, connection_str, timeout, logger_);
      } break;

      case TCP_DATA_SOURCE: {
        std::string ip;
        uint16_t port;
        uint32_t timeout;
        get_parameter<std::string>("host_ip", ip);
        get_parameter<uint16_t>("host_port", port);
        get_parameter<uint32_t>("timeout", timeout);
        reader_ = dataSourceFactory(ip, port, timeout, logger_);
      } break;

      default:
        LOG_FATAL(logger_,
                  "Could not create a data source of type: " << interface);
        break;
    }
  }

  /**
   * @brief ROS 2 parameters declaration methods
   */
  void declareParameters() {
    declare_parameter<std::string>("host_ip", "");
    declare_parameter<uint16_t>("host_port", 0);
    declare_parameter<uint8_t>("interface", 0);
    declare_parameter<std::string>("sbp_file", "");
    declare_parameter<bool>("navsatfix", true);
    declare_parameter<bool>("timereference", true);
  }

  /**
   * @brief Method for creating the ROS 2 publishers
   */
  void createPublishers() {
    bool enabled;

    get_parameter<bool>("navsatfix", enabled);
    auto navsatfix_publisher =
        create_publisher<sensor_msgs::msg::NavSatFix>("navsatfix", 10);
    navsatfix_publisher_ = std::make_unique<NavSatFixPublisher>(
        &state_, navsatfix_publisher, this, enabled);

    get_parameter<bool>("timereference", enabled);
    auto timereference_publisher =
        create_publisher<sensor_msgs::msg::TimeReference>("timereference", 10);
    timereference_publisher_ = std::make_unique<TimeReferencePublisher>(
        &state_, timereference_publisher, this, enabled);
  }

  sbp::State state_;           /** @brief SBP state object */
  std::thread sbp_thread_;     /** @brief SBP messages processing thread */
  bool exit_requested_{false}; /** @brief Thread stopping flag */
  std::shared_ptr<sbp::IReader> reader_; /** @brief data source object */
  std::shared_ptr<ROSLogger> logger_;    /** @brief ROS 2 logging object */
  std::unique_ptr<NavSatFixPublisher>
      navsatfix_publisher_; /** @brief NavSatFix ROS 2 publisher */
  std::unique_ptr<TimeReferencePublisher>
      timereference_publisher_; /** @brief TimeReference ROS 2 publisher */
};

int main(int argc, char** argv) {
  printf("hello world swiftnav-ros2 package\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SBPROS2DriverNode>());
  rclcpp::shutdown();

  return 0;
}
