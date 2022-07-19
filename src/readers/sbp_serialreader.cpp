#include <readers/sbp_serialreader.h>

SBPSerialReader::SBPSerialReader(const std::string& connection_string,
                                 const LoggerPtr& logger) noexcept
    : logger_(logger) {}

SBPSerialReader::SBPSerialReader(SBPSerialReader&& rhs) noexcept {
  port_ = rhs.port_;
  rhs.port_ = nullptr;
  logger_ = rhs.logger_;
  rhs.logger_.reset();
}

SBPSerialReader::~SBPSerialReader() { closePort(); }

s32 SBPSerialReader::read(u8* buffer, u32 buffer_length) {
  if (!port_) {
    LOG_FATAL(logger_, "Called read in an uninitialized SBPSerialReader");
    return -1;
  } else if (!buffer) {
    LOG_FATAL(logger_, "Called SBPSerialReader::read with a NULL buffer");
    return -1;
  }
}

void SBPSerialReader::closePort() noexcept {
  if (port_) {
    std::string port_name(sp_get_port_name(port_));
    if (sp_close(port_) != SP_OK) {
      LOG_ERROR(logger_, "Could not close " << port_name);
    } else {
      LOG_INFO(logger_, port_name << " closed");
    }

    sp_free_port(port_);
    port_ = nullptr;
  }
}
