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

#include <libsbp/cpp/state.h>

/**
 * @brief Base class for Data Sources
 */
class SbpDataSource : public sbp::IReader, public sbp::IWriter {
 public:
  /**
   * @brief Method to read data from the connection
   *
   * @param buffer Buffer to save the readed data. It must be long enough to
   * contain buffer_length bytes
   * @param buffer_length Max number of bytes to read
   * @return Number of bytes actually readed
   */
  s32 read(u8* /*buffer*/, u32 /*buffer_length*/) override { return -1; }

  /**
   * @brief Method to write data to the connection
   *
   * @param buffer Buffer containing the data to write
   * @param buffer_length Number of bytes to write
   * @return Number of bytes actually written
   */
  s32 write(const u8* /*buffer*/, u32 /*buffer_length*/) override { return -1; }
};
