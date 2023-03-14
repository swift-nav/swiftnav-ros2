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

#include <data_sources/sbp_file_datasource.h>

SbpFileDataSource::SbpFileDataSource(const std::string &file_path,
                                     const LoggerPtr &logger)
    : logger_(logger) {
  file_stream_ = std::ifstream(file_path, std::ios::binary | std::ios_base::in);
  if (!file_stream_.is_open())
    LOG_FATAL(logger_, "File: %s  couldn't be open", file_path.c_str());
}

SbpFileDataSource::~SbpFileDataSource() { file_stream_.close(); }

bool SbpFileDataSource::eof() const {
  if (file_stream_.is_open())
    return file_stream_.eof();
  else
    return true;
}

s32 SbpFileDataSource::read(u8 *buffer, u32 buffer_length) {
  auto start_index = file_stream_.tellg();
  if (start_index == -1) {
    return -1;
  }
  file_stream_.read(reinterpret_cast<char *>(buffer), buffer_length);
  auto end_index = file_stream_.tellg();
  if (end_index == -1 || file_stream_.fail()) {
    return -1;
  }

  return static_cast<s32>(end_index - start_index);
}
