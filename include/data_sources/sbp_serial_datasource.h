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
#include <data_sources/serial.h>
#include <logging/issue_logger.h>
#include <memory>
#include <string>

/**
 * @brief Class that implements a Serial Port reader based on the SBP reader
 * interface
 */
class SbpSerialDataSource : public SbpDataSource {
 public:
  /**
   * @brief Construct a new SbpSerialDataSource object
   *
   * @param logger Logger facility to use
   * @param serial A serial communications object
   */
  SbpSerialDataSource(const LoggerPtr& logger,
                      const std::shared_ptr<SerialPort>& serial) noexcept;

  // Deleted methods
  SbpSerialDataSource() = delete;
  SbpSerialDataSource(const SbpSerialDataSource& rhs) = delete;

  /**
   * @brief Method to read data from the serial connection
   *
   * @param buffer Buffer to save the readed data. It must be long enough to
   * contain buffer_length bytes
   * @param buffer_length Max number of bytes to read
   * @return Number of bytes actually readed
   */
  s32 read(u8* buffer, u32 buffer_length) override;

  /**
   * @brief Method to write data to the serial connection
   *
   * @param buffer Buffer containing the data to write
   * @param buffer_length Number of bytes to write
   * @return Number of bytes actually written
   */
  s32 write(const u8* buffer, u32 buffer_length) override;

  /**
   * @brief Determines if the object is valid
   *
   * @return true Object is valid
   * @return false Object isn't valid
   */
  bool isValid() const noexcept;

 private:
  std::shared_ptr<SerialPort> port_; /** @brief Serial port object */
  LoggerPtr logger_;                 /** @brief Logging facility */
};
