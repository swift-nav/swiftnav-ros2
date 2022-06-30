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
#include <logging/issue_logger.h>
#include <fstream>
#include <string>

class SbpFileDataSource : public SbpDataSource {
 public:
  SbpFileDataSource() = delete;

  /**
   * @brief Construct a new Sbp File Data Source object
   *
   * @param file_path Path to the SBP file to use
   * @param logger Logging facility
   */
  SbpFileDataSource(const std::string &file_path, const LoggerPtr &logger);

  /**
   * @brief Destroy the Sbp File Data Source object
   */
  ~SbpFileDataSource();

  /**
   * @brief Methor to determine if the file is open
   *
   * @return true File opened
   * @return false File closed
   */
  bool is_open() const { return file_stream_.is_open(); }

  /**
   * @brief Method to determine if the reading operation has reached the EOF
   *
   * @return true EOF reached
   * @return false EOF not reached
   */
  bool eof() const;

  /**
   * @brief Overriden method to read from the file
   *
   * @param buffer Buffer where to put the data readed
   * @param buffer_length Number of bytes to read
   * @return Number of bytes actually readed
   */
  s32 read(u8 *buffer, u32 buffer_length) override;

 private:
  std::ifstream file_stream_; /** @brief File stream representing the file */
  LoggerPtr logger_;          /** @brief Logging facility object */
};
