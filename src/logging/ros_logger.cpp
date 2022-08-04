#include <logging/ros_logger.h>
#include <rclcpp/rclcpp.hpp>

void ROSLogger::logDebug(const std::stringstream& ss) {
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), ss.str().c_str());
}

void ROSLogger::logInfo(const std::stringstream& ss) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ss.str().c_str());
}

void ROSLogger::logWarning(const std::stringstream& ss) {
  RCLCPP_WARN(rclcpp::get_logger("rclcpp"), ss.str().c_str());
}

void ROSLogger::logError(const std::stringstream& ss) {
  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), ss.str().c_str());
}

void ROSLogger::logFatal(const std::stringstream& ss) {
  RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), ss.str().c_str());
}
