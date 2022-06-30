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
  if (!buffer) {
    LOG_ERROR(logger_, "Buffer passed to SbpTCPDataSource::read is NULL");
    return -1;
  }

  if (!isValid()) {
    LOG_ERROR(logger_,
              "Read operation requested on an uninitialized SbpTCPDataSource");
    return -1;
  }

  return tcp_->read(buffer, buffer_length);
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
