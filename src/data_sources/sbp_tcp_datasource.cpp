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

#include <data_sources/sbp_tcp_datasource.h>
#include <cstring>
#include <iostream>

SbpTCPDataSource::SbpTCPDataSource(const LoggerPtr& logger,
                                   const std::shared_ptr<TCP>& tcp) noexcept
    : tcp_(tcp), logger_(logger) {
  if (!tcp_) {
    // LOG_FATAL(logger_, "No TCP object attached");
    return;
  }

  if (!tcp_->open()) LOG_FATAL(logger_, "Could not establish a TCP connection");
}

s32 SbpTCPDataSource::read(u8* buffer, u32 buffer_length) {
  int32_t read_bytes = 0;
  if (!buffer) {
    LOG_ERROR(logger_, "Buffer passed to SbpTCPDataSource::read is NULL");
    return -1;
  }

  if (!isValid()) {
    LOG_ERROR(logger_,
              "Read operation requested on an uninitialized SbpTCPDataSource");
    return -1;
  }

  read_bytes = tcp_->read(buffer, buffer_length);

  // Attempt to reconnect on error
  if (-1 == read_bytes){
      tcp_->close();
      tcp_->open();
  }
  return read_bytes;
}

s32 SbpTCPDataSource::write(const u8* buffer, u32 buffer_length) {
  if (!buffer) {
    LOG_ERROR(logger_, "Buffer passed to SbpTCPDataSource::write is NULL");
    return -1;
  }

  if (!isValid()) {
    LOG_ERROR(logger_,
              "Write operation requested on an uninitialized SbpTCPDataSource");
    return -1;
  }

  return tcp_->write(buffer, buffer_length);
}

bool SbpTCPDataSource::isValid() const noexcept {
  if (tcp_)
    return tcp_->isValid();
  else
    return false;
}
