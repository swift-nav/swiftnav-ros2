/*
 * Copyright (C) 2015-2023 Swift Navigation Inc.
 * Contact: https://support.swiftnav.com
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <data_sources/serial.h>
#include <sstream>
#include <vector>

/**
 * @brief Auxiliary class to split the connection string
 */
class SerialParameterSplitter {
 public:
  uint32_t speed{0U};     /** @brief Baud rate */
  uint32_t data_bits{0U}; /** @brief Number of data bits */
  uint32_t stop_bits{0U}; /** @brief Number of stop bits */
  char parity{'-'};       /** @brief Parity control type */
  char flow_control{'-'}; /** @brief Flow control type */

  /**
   * @brief Construct a new Serial Parameter Splitter object
   *
   * @param str String to decompose inj tokens
   * @param logger Logger facility to use
   */
  SerialParameterSplitter(const std::string& str,
                          const LoggerPtr& logger) noexcept {
    split(str);
    ASSERT_COND(token_list_.size() == 5U, logger, "Malformed string");
    setValues();
  }

  /**
   * @brief Method used to determine if the values in the object are valid
   *
   * @return true The values are valid
   * @return false The values are not valid
   */
  bool isValid() const noexcept {
    // Test speed
    switch (speed) {
      case 1200:
      case 2400:
      case 4800:
      case 9600:
      case 19200:
      case 38400:
      case 57600:
      case 115200:
      case 230400:
      case 460800:
      case 921600:
        break;

      default:
        return false;
        break;
    }

    // Test data bits
    if (data_bits < 7 || data_bits > 8) return false;

    // Test stop bits
    if (stop_bits < 1 || stop_bits > 2) return false;

    // Test parity
    switch (parity) {
      case 'N':
      case 'E':
      case 'O':
      case 'M':
      case 'S':
        break;

      default:
        return false;
        break;
    }

    // Test flow control
    switch (flow_control) {
      case 'N':
      case 'X':
      case 'R':
      case 'D':
        break;

      default:
        return false;
        break;
    }

    return true;
  }

 private:
  /**
   * @brief Method used to split the string into a vector of tokens
   *
   * @param str String composed by tokens
   */
  void split(const std::string& str) noexcept {
    try {
      std::stringstream ss(str);
      std::string token;

      token_list_.clear();
      while (std::getline(ss, token, '|')) {
        token_list_.emplace_back(token);
      }
    } catch (...) {
      token_list_.clear();
    }
  }

  /**
   * @brief Set the Values from the tokens to the corresponding variables
   */
  void setValues() noexcept {
    try {
      speed = static_cast<uint32_t>(std::stoul(token_list_[0]));
      parity = static_cast<char>(std::toupper(token_list_[1][0]));
      data_bits = static_cast<uint32_t>(std::stoul(token_list_[2]));
      stop_bits = static_cast<uint32_t>(std::stoul(token_list_[3]));
      flow_control = static_cast<char>(std::toupper(token_list_[4][0]));
    } catch (...) {
      speed = 0U;
      parity = 'N';
      data_bits = 0U;
      stop_bits = 0U;
      flow_control = 'N';
    }

    token_list_.clear();
  }

  std::vector<std::string> token_list_;
};

SerialPort::SerialPort(const std::string& device_name,
                       const std::string& connection_string,
                       const uint32_t read_timeout,
                       const uint32_t write_timeout, const LoggerPtr& logger)
    : logger_(logger),
      device_name_(device_name),
      connection_string_(connection_string),
      read_timeout_(read_timeout),
      write_timeout_(write_timeout) {}

SerialPort::~SerialPort() { closePort(); }

bool SerialPort::open() noexcept {
  if (device_name_.empty()) {
    LOG_FATAL(logger_, "The port name should be specified");
    return false;
  }

  sp_return result = sp_get_port_by_name(device_name_.c_str(), &port_);
  if (result != SP_OK) {
    LOG_FATAL(logger_, "No serial port named %s", device_name_.c_str());
    return false;
  }

  result = sp_open(port_, SP_MODE_READ_WRITE);
  if (result != SP_OK) {
    LOG_FATAL(logger_, "Cannot open port : %s. Error: %d",
              sp_get_port_name(port_), result);
    return false;
  }

  SerialParameterSplitter params(connection_string_, logger_);
  if (!params.isValid()) {
    LOG_FATAL(logger_, "Invalid data in connection string: %s",
              connection_string_.c_str());
    return false;
  }

  const std::string error = setPortSettings(params);
  if (!error.empty()) {
    LOG_FATAL(logger_, error.c_str());
    return false;
  }

  sp_flush(port_, SP_BUF_BOTH);
  LOG_INFO(
      logger_,
      "Port %s opened with:\nBaud rate: %u\nParity: %c\nData bits: %u\nStop "
      "bits: %u\nFlow control: %c",
      device_name_.c_str(), params.speed, params.parity, params.data_bits,
      params.stop_bits, params.flow_control);
  return true;
}

int32_t SerialPort::read(uint8_t* buffer, const uint32_t buffer_length) {
  if (!port_) {
    LOG_FATAL(logger_, "Called read in an uninitialized SbpSerialDataSource");
    return -1;
  }

  if (!buffer) {
    LOG_FATAL(logger_, "Called SerialPort::read with a NULL buffer");
    return -1;
  }

  const auto result = sp_nonblocking_read(port_, buffer, buffer_length);
  if (result < 0) {
    LOG_ERROR(logger_, "Error (%d) while reading the serial port", result);
    return -1;
  }

  return result;
}

int32_t SerialPort::write(const uint8_t* buffer, const uint32_t buffer_length) {
  if (!port_) {
    LOG_ERROR(logger_, "Called write in an uninitialized SbpSerialDataSource");
    return -1;
  }

  if (!buffer) {
    LOG_ERROR(logger_, "Called SerialPort::write with a NULL buffer");
    return -1;
  }

  const auto result =
      sp_blocking_write(port_, buffer, buffer_length, write_timeout_);
  if (result < 0) {
    LOG_ERROR(logger_, "Error (%d) while writing to the serial port", result);
    return -1;
  }

  return result;
}

bool SerialPort::isValid() const noexcept { return (port_) ? true : false; }

std::string SerialPort::setPortSettings(
    const SerialParameterSplitter& params) noexcept {
  sp_return result;

  sp_flowcontrol flow_control;
  switch (params.flow_control) {
    case 'N':
      flow_control = SP_FLOWCONTROL_NONE;
      break;

    case 'X':
      flow_control = SP_FLOWCONTROL_XONXOFF;
      break;

    case 'R':
      flow_control = SP_FLOWCONTROL_RTSCTS;
      break;

    case 'D':
      flow_control = SP_FLOWCONTROL_DTRDSR;
      break;

    default:
      flow_control = SP_FLOWCONTROL_NONE;
      break;
  }

  result = sp_set_flowcontrol(port_, flow_control);
  if (result != SP_OK)
    return (std::string("Cannot set flow control: ") + std::to_string(result));

  result = sp_set_bits(port_, params.data_bits);
  if (result != SP_OK)
    return (std::string("Cannot set data bits: ") + std::to_string(result));

  sp_parity parity;
  switch (params.parity) {
    case 'N':
      parity = SP_PARITY_NONE;
      break;

    case 'O':
      parity = SP_PARITY_ODD;
      break;

    case 'E':
      parity = SP_PARITY_EVEN;
      break;

    case 'M':
      parity = SP_PARITY_MARK;
      break;

    case 'S':
      parity = SP_PARITY_SPACE;
      break;

    default:
      parity = SP_PARITY_INVALID;
      break;
  }

  result = sp_set_parity(port_, parity);
  if (result != SP_OK)
    return (std::string("Cannot set parity: ") + std::to_string(result));

  result = sp_set_stopbits(port_, params.stop_bits);
  if (result != SP_OK)
    return (std::string("Cannot set stop bits: ") + std::to_string(result));

  result = sp_set_baudrate(port_, params.speed);
  if (result != SP_OK)
    return (std::string("Cannot set baud rate: ") + std::to_string(result));

  return {};
}

void SerialPort::closePort() noexcept {
  if (port_) {
    std::string port_name(sp_get_port_name(port_));
    if (sp_close(port_) != SP_OK) {
      LOG_ERROR(logger_, "Could not close %s", port_name.c_str());
    } else {
      LOG_INFO(logger_, "%s closed", port_name.c_str());
    }

    sp_free_port(port_);
    port_ = nullptr;
  }
}
