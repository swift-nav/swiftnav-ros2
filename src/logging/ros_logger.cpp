#include <logging/ros_logger.h>
#include <rclcpp/rclcpp.hpp>

void ROSLogger::logDebug(const std::stringstream& ss) {
  const std::string output(ss.str());
  if (canLog(output)) {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), output.c_str());
    updateLogStatus(output);
  }
}

void ROSLogger::logInfo(const std::stringstream& ss) {
  const std::string output(ss.str());
  if (canLog(output)) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), output.c_str());
    updateLogStatus(output);
  }
}

void ROSLogger::logWarning(const std::stringstream& ss) {
  const std::string output(ss.str());
  if (canLog(output)) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), output.c_str());
    updateLogStatus(output);
  }
}

void ROSLogger::logError(const std::stringstream& ss) {
  const std::string output(ss.str());
  if (canLog(output)) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), output.c_str());
    updateLogStatus(output);
  }
}

void ROSLogger::logFatal(const std::stringstream& ss) {
  const std::string output(ss.str());
  if (canLog(output)) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), output.c_str());
    updateLogStatus(output);
  }
}

bool ROSLogger::canLog(const std::string_view output_str) const {
  if (output_str == last_output_str_) {
    const auto now =
        std::chrono::time_point<std::chrono::system_clock>::clock::now();
    return ((now - last_output_time_).count() >= timeout_);
  } else {
    return true;
  }
}

void ROSLogger::updateLogStatus(const std::string_view output_str) {
  last_output_str_ = output_str;
  last_output_time_ =
      std::chrono::time_point<std::chrono::system_clock>::clock::now();
}

ROSLogger::ROSLogger(const int64_t log_delay) : timeout_(log_delay) {}
