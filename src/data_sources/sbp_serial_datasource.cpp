#include <data_sources/sbp_serial_datasource.h>

SbpSerialDataSource::SbpSerialDataSource(
    const LoggerPtr& logger, std::unique_ptr<SerialPort> serial) noexcept
    : logger_(logger) {
  if (serial) {
    port_ = std::move(serial);
  } else {
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
