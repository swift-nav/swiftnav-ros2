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

#include <logging/issue_logger.h>
#include <cstdint>
#include <memory>
#include <string>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#endif  // _WIN32

/**
 * @brief Class that isolates the OS TCP implementation from the driver
 */
class TCP {
 public:
  TCP() = delete;

  /**
   * @brief Construct a new TCP object
   *
   * @param ip Ip address to connect to
   * @param port IP port to connect to
   * @param logger Logging facility
   * @param read_timeout Timeout in ms for the read operation to succeed
   * @param write_timeout Timeout in ms for the write operation to succeed
   */
  TCP(const std::string& ip, const uint16_t port, const LoggerPtr& logger,
      const uint32_t read_timeout, const uint32_t write_timeout);

  /**
   * @brief Destroy the TCP object
   */
  virtual ~TCP();

  /**
   * @brief Opens the TCP connection
   *
   * @return true The TCP connection could be opened
   * @return false The TCP connection couldn't be opened
   */
  virtual bool open() noexcept;

  /**
   * @brief Closes the connection
   */
  virtual void close() noexcept;

  /**
   * @brief Read bytes from the TCP connection
   *
   * @param buffer Buffer where to put the read bytes
   * @param buffer_size Number of bytes to read (up to buffer size)
   * @return Number of bytes actually read
   */
  virtual int32_t read(uint8_t* buffer, const uint32_t buffer_size);

  /**
   * @brief Write bytes to the TCP connection
   *
   * @param buffer Buffer from where to write the bytes
   * @param buffer_size Number of bytes to write (up to buffer size)
   * @return Number of bytes actually written
   */
  virtual int32_t write(const uint8_t* buffer, const uint32_t buffer_size);

  /**
   * @brief Determines if the object is valid (valid TCP connection) or not
   *
   * @return true The object is valid
   * @return false The object isn't valid
   */
  virtual bool isValid() const noexcept;

 protected:
  /**
   * @brief Method to init the socket system.
   *
   * @return String containig the error. Empty if OK
   */
  std::string initSockets() noexcept;

  /**
   * @brief Method to clean and free socket system resources
   */
  void deinitSockets() noexcept;

  /**
   * @brief Closes the open socket in use
   */
  void closeSocket() noexcept;

  /**
   * @brief Method that opens and connects the socket
   */
  bool openSocket() noexcept;

  /**
   * @brief Sets the socket to be non-blocking
   *
   * @return true The socket could be configured
   * @return false The configuration failed
   */
  bool setNonBlocking() noexcept;

  /**
   * @brief Method to connect the socket client to the server
   *
   * @return true The socket is connected
   * @return false The socket failed to connect
   */
  bool connectSocket() noexcept;

#if defined(_WIN32)
  SOCKET socket_id_{INVALID_SOCKET}; /** @brief Windows Socket */
#else
  int socket_id_{-1}; /** @brief Linux socket */
#endif  // _WIN32

  LoggerPtr logger_;           /** @brief Logging facility */
  std::string ip_;             /** @brief IP to connect to */
  uint16_t port_;              /** @brief TCP port to connect to */
  uint32_t read_timeout_;      /** @brief Read timeout in ms */
  uint32_t write_timeout_;     /** @brief Write timeout in ms */
};