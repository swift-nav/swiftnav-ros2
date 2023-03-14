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

#pragma once

#include <libserialport.h>
#include <logging/issue_logger.h>
#include <cstdint>
#include <string>
#include <string_view>

/**
 * @brief Class of serial port interface
 */
class SerialPort {
 public:
  SerialPort() = delete;

  /**
   * @brief Construct a new Serial Port object
   *
   * @param device_name String containing the name of the serial port to use
   * @param connection_string String containing the data needed to open the
   * serial port. The format of the string is: SPEED|DATA BITS|PARITY|STOP
   * BITS|FLOW CONTROL, where: \nSPEED: 9600, or 19200, or 115200, etc.\nDATA
   * BITS: Number of data bits (8 for example)\nPARITY: Parity control (N: none,
   * O: Odd, E: even, M: mark, S: space)\nSTOP BITS: Number of stop bits used
   * (1, 2)\nFLOW CONTROL: (N: none, X: Xon/Xoff, R: RTS/CTS, D: DTR/DSR)
   * Example: 19200|N|8|1|N
   * @param read_timeout Timeout in milliseconds for the read operation to
   * complete. If 0 (default) the read operation blocks until the requested
   * number of bytes is read or an error occurs
   * @param write_timeout Timeout in milliseconds for the write operation to
   * complete. If 0 (default) the write operation blocks until the requested
   * number of bytes is written or an error occurs
   * @param logger Logging facility
   */
  SerialPort(const std::string& device_name,
             const std::string& connection_string, const uint32_t read_timeout,
             const uint32_t write_timeout, const LoggerPtr& logger);

  /**
   * @brief Destroy the Serial Port object
   */
  virtual ~SerialPort();

  /**
   * @brief Opens the port and configures it with the supplied setting (from
   * connection string)
   *
   * @return true The port is open and functional
   * @return false The port can't be used.
   */
  virtual bool open() noexcept;

  /**
   * @brief Reads bytes from the port
   *
   * @param buffer Buffer where to store the read bytes
   * @param buffer_length Buffer size (max number of bytes to read)
   * @return Number of bytes actually read
   */
  virtual int32_t read(uint8_t* buffer, const uint32_t buffer_length);

  /**
   * @brief Writes bytes to the port
   *
   * @param buffer Buffer where the bytes are
   * @param buffer_length Number of bytes to write
   * @return Number of bytes actually written
   */
  virtual int32_t write(const uint8_t* buffer, const uint32_t buffer_length);

  /**
   * @brief Determines if the object is usable or not
   *
   * @return true The object is usable
   * @return false The object is not usable
   */
  virtual bool isValid() const noexcept;

 protected:
  /**
   * @brief Method to configure the port
   *
   * @param params Object containing the parameters to set
   * @return String containing the error. Empty if OK
   */
  std::string setPortSettings(
      const class SerialParameterSplitter& params) noexcept;

  /**
   * @brief Method to close the port and release the resources
   */
  void closePort() noexcept;

  sp_port* port_;    /** @brief Pointer to a libserialport structure
                       representing a port */
  LoggerPtr logger_; /** @brief Logging facility */
  std::string device_name_;       /** @brief Name of the serial device */
  std::string connection_string_; /** @brief String containing the serial port
                                     parametrization */
  uint32_t read_timeout_{0U};  /** @brief read timeout in ms */
  uint32_t write_timeout_{0U}; /** @brief write timeout in ms */
};
