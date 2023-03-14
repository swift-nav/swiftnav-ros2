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

#include <data_sources/sbp_serial_datasource.h>

SbpSerialDataSource::SbpSerialDataSource(
    const LoggerPtr& logger, const std::shared_ptr<SerialPort>& serial) noexcept
    : port_(serial), logger_(logger) {
  if (!port_) {
    LOG_FATAL(logger_, "No serial port attached");
    return;
  }

  if (!port_->open()) LOG_FATAL(logger_, "Serial port can't be used");
}

s32 SbpSerialDataSource::read(u8* buffer, u32 buffer_length) {
  if (!isValid()) {
    LOG_FATAL(logger_,
              "Called read in an uninitialized SbpSerialDataSource");
    return -1;
  } else {
    return port_->read(buffer, buffer_length);
  }
}

s32 SbpSerialDataSource::write(const u8* buffer, u32 buffer_length) {
  if (!isValid()) {
    LOG_FATAL(logger_,
              "Called write in an uninitialized SbpSerialDataSource");
    return -1;
  } else {
    return port_->write(buffer, buffer_length);
  }
}

bool SbpSerialDataSource::isValid() const noexcept {
  return (port_ && port_->isValid());
}
