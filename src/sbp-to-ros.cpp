#include <chrono>
#include <functional>
#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <logging/ros_logger.h>
#include <logging/sbp_to_ros2_logger.h>

#include <libsbp/cpp/state.h>
#include <libsbp/cpp/message_handler.h>

#include <publishers/GPSFixPublisher.h>
#include <publishers/PoseStampedPublisher.h>
#include <publishers/NavSatFixPublisher.h>
#include <publishers/TimeReferencePublisher.h>
#include <publishers/angular_rate_publisher.h>
#include <publishers/baseline_heading_publisher.h>
#include <publishers/orient_euler_publisher.h>
#include <publishers/orient_quat_publisher.h>

//#include <subscribers/IMUSubscriber.h>

#include <data_sources/sbp_data_sources.h>
#include <utils.h>

static const int64_t LOG_REPUBLISH_DELAY = 2_ns;

/**
 * @brief Class that represents the ROS 2 driver node
 */
class SBPROS2DriverNode : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new SBPROS2DriverNode object
   */
  SBPROS2DriverNode() : Node("SBPRos2Driver") {
    declareParameters();
    get_parameter<std::string>("frame_name", frame_);
    logger_ = std::make_shared<ROSLogger>(LOG_REPUBLISH_DELAY);

    createDataSources();
    if (!data_source_) exit(EXIT_FAILURE);
    state_.set_reader(data_source_.get());
    state_.set_writer(data_source_.get());
    createPublishers();
    createSubscribers();

    bool log_sbp_messages;
    std::string log_path;

    get_parameter<bool>("log_sbp_messages", log_sbp_messages);
    get_parameter<std::string>("log_sbp_filepath", log_path);
    sbptoros2_ = std::make_shared<SBPToROS2Logger>(&state_, logger_,
                                                   log_sbp_messages, log_path);

    /* SBP Callback processing thread */
    sbp_thread_ = std::thread(&SBPROS2DriverNode::processSBP, this);
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
   * @brief Method for creating the data sources
   */
  void createDataSources() {
    int32_t interface;

    get_parameter<int32_t>("interface", interface);
    switch (interface) {
      case FILE_DATA_SOURCE: {
        std::string file;
        get_parameter<std::string>("sbp_file", file);
        data_source_ = dataSourceFactory(file, logger_);
      } break;

      case SERIAL_DATA_SOURCE: {
        std::string device, connection_str;
        int32_t read_timeout, write_timeout;
        get_parameter<std::string>("device_name", device);
        get_parameter<std::string>("connection_str", connection_str);
        get_parameter<int32_t>("read_timeout", read_timeout);
        get_parameter<int32_t>("write_timeout", write_timeout);
        data_source_ = dataSourceFactory(device, connection_str, read_timeout,
                                         write_timeout, logger_);
      } break;

      case TCP_DATA_SOURCE: {
        std::string ip;
        int32_t port, read_timeout, write_timeout;
        get_parameter<std::string>("host_ip", ip);
        get_parameter<int32_t>("host_port", port);
        get_parameter<int32_t>("read_timeout", read_timeout);
        get_parameter<int32_t>("write_timeout", write_timeout);
        data_source_ =
            dataSourceFactory(ip, port, read_timeout, write_timeout, logger_);
      } break;

      default:
        LOG_FATAL(logger_, "Could not create a data source of type: %d",
                  interface);
        break;
    }
  }

  /**
   * @brief ROS 2 parameters declaration methods
   */
  void declareParameters() {
    declare_parameter<int32_t>("interface", 0);
    declare_parameter<std::string>("sbp_file", "");
    declare_parameter<std::string>("device_name", "");
    declare_parameter<std::string>("connection_str", "");
    declare_parameter<std::string>("host_ip", "");
    declare_parameter<int32_t>("host_port", 0);
    declare_parameter<int32_t>("read_timeout", 0);
    declare_parameter<int32_t>("write_timeout", 0);
    declare_parameter<bool>("navsatfix", true);
    declare_parameter<bool>("timereference", true);
    declare_parameter<bool>("gpsfix", true);
    declare_parameter<bool>("posestamped", true);
    declare_parameter<bool>("baseline_heading", true);
    declare_parameter<bool>("angular_rate", true);
    declare_parameter<bool>("orient_euler", true);
    declare_parameter<bool>("orient_quat", true);
    declare_parameter<bool>("log_sbp_messages", false);
    declare_parameter<std::string>("log_sbp_filepath", "");
    declare_parameter<std::string>("frame_name", "gps");
  }

  /**
   * @brief Method for creating the ROS 2 publishers
   */
  void createPublishers() {
    bool enabled;

    get_parameter<bool>("navsatfix", enabled);
    navsatfix_publisher_ = std::make_unique<NavSatFixPublisher>(
        &state_, "navsatfix", this, logger_, enabled, frame_);

    get_parameter<bool>("timereference", enabled);
    timereference_publisher_ = std::make_unique<TimeReferencePublisher>(
        &state_, "timereference", this, logger_, enabled, frame_);

    get_parameter<bool>("gpsfix", enabled);
    gpsfix_publisher_ = std::make_unique<GPSFixPublisher>(
        &state_, "gpsfix", this, logger_, enabled, frame_);

    get_parameter<bool>("posestamped", enabled);
    posestamped_publisher_ = std::make_unique<PoseStampedPublisher>(
        &state_, "posestamped", this, logger_, enabled, frame_);

    // SBP to ROS custom messages
    get_parameter<bool>("baseline_heading", enabled);
    baseline_heading_publisher_ = std::make_unique<BaselineHeadingPublisher>(
        &state_, "baseline_heading", this, logger_, enabled, frame_);

    get_parameter<bool>("angular_rate", enabled);
    angular_rate_publisher_ = std::make_unique<AngularRatePublisher>(
        &state_, "angular_rate", this, logger_, enabled, frame_);

    get_parameter<bool>("orient_euler", enabled);
    orient_euler_publisher_ = std::make_unique<OrientEulerPublisher>(
        &state_, "orient_euler", this, logger_, enabled, frame_);

    get_parameter<bool>("orient_quat", enabled);
    orient_quat_publisher_ = std::make_unique<OrientQuatPublisher>(
        &state_, "orient_quat", this, logger_, enabled, frame_);
  }

  /**
   * @brief Method for creating ROS 2 subscribers
   */
  void createSubscribers() {}

  sbp::State state_;           /** @brief SBP state object */
  std::thread sbp_thread_;     /** @brief SBP messages processing thread */
  bool exit_requested_{false}; /** @brief Thread stopping flag */
  std::shared_ptr<SbpDataSource> data_source_; /** @brief data source object */
  std::shared_ptr<ROSLogger> logger_;    /** @brief ROS 2 logging object */
  std::unique_ptr<NavSatFixPublisher>
      navsatfix_publisher_; /** @brief NavSatFix ROS 2 publisher */
  std::unique_ptr<TimeReferencePublisher>
      timereference_publisher_; /** @brief TimeReference ROS 2 publisher */
  std::unique_ptr<GPSFixPublisher>
      gpsfix_publisher_; /** @brief GPSFix ROS 2 publisher */
  std::unique_ptr<PoseStampedPublisher>
      posestamped_publisher_; /** @brief PoseStamped ROS 2 publisher */
  std::unique_ptr<BaselineHeadingPublisher>
      baseline_heading_publisher_; /** @brief MSG_BASELINE_HEADING publisher */
  std::unique_ptr<AngularRatePublisher>
      angular_rate_publisher_; /** @brief MSG_ANGULAR_RATE publisher */
  std::unique_ptr<OrientEulerPublisher>
      orient_euler_publisher_; /** @brief MSG_ORIENT_EULER publisher */
  std::unique_ptr<OrientQuatPublisher>
      orient_quat_publisher_; /** @brief MSG_ORIENT_QUAT publisher */
  std::shared_ptr<SBPToROS2Logger>
      sbptoros2_; /** @brief SBP to ROS2 logging object */
  std::string frame_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SBPROS2DriverNode>());
  rclcpp::shutdown();

  return 0;
}
