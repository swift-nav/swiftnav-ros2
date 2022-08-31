#pragma once

#include <libserialport.h>
#include <logging/issue_logger.h>
#include <cstdint>
#include <string>
#include <string_view>

class SerialPort {
 public:
  SerialPort() = delete;

  SerialPort(const std::string& device_name,
             const std::string& connection_string, const uint32_t read_timeout,
             const uint32_t write_timeout, const LoggerPtr& logger);

  virtual bool open() noexcept;
  virtual int32_t read(uint8_t* buffer, const uint32_t buffer_length);
  virtual int32_t write(const uint8_t* buffer, const uint32_t buffer_length);
  virtual bool isValid() const noexcept;

 private:
  /**
   * @brief Method to configure the port
   *
   * @param params Object containing the parameters to set
   * @return String containing the error. Empty if OK
   */
  std::string setPortSettings(
      const class SerialParameterSplitter& params) noexcept;

  /**
   * @brief Method to close the port and release the resources
   */
  void closePort() noexcept;

  sp_port* port_;    /** @brief Pointer to a libserialport structure
                       representing a port */
  LoggerPtr logger_; /** @brief Logging facility */
  std::string device_name_;
  std::string connection_string_;
  uint32_t read_timeout_{0U};  /** @brief read timeout in ms */
  uint32_t write_timeout_{0U}; /** @brief write timeout in ms */
};
