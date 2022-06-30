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

#include <data_sources/sbp_data_source.h>
#include <data_sources/tcp.h>
#include <memory>

/**
 * @brief Class that implements a TCP reader based on the SBP reader interface
 * (IReader)
 */
class SbpTCPDataSource : public SbpDataSource {
 public:
  /**
   * @brief Construct a new SbpTCPDataSource object
   *
   * @param ip IP address to connect to. It could be in IPV4 or IPV6 format
   * @param port TCP port to connect to
   * @param logger Logger facility to use
   * @param read_timeout Timeout in ms for the read operation to start. If 0,
   * then the read operation blocks until the requested number of bytes have
   * read or an error ocurred
   */
  SbpTCPDataSource(const LoggerPtr& logger,
                   const std::shared_ptr<TCP>& tcp) noexcept;

  // Deleted methods
  SbpTCPDataSource() = delete;

  /**
   * @brief Method to read data from the TCP connection
   *
   * @param buffer Buffer to save the readed data. It must be long enough to
   * contain buffer_length bytes
   * @param buffer_length Max number of bytes to read
   * @return Number of bytes actually readed
   */
  s32 read(u8* buffer, u32 buffer_length) override;

  /**
   * @brief Method to write data to the TCP connection
   *
   * @param buffer Buffer containing the data to write
   * @param buffer_length Number of bytes to write
   * @return Number of bytes actually written
   */
  s32 write(const u8* buffer, u32 buffer_length) override;

  /**
   * @brief Method to determine if the internal socket is valid or not
   *
   * @return true Socket is valid
   * @return false Socket is not valid
   */
  bool isValid() const noexcept;

 private:
  std::shared_ptr<TCP> tcp_; /** @brief TCP/IP data object */
  LoggerPtr logger_;         /** @brief Logging facility */
};
