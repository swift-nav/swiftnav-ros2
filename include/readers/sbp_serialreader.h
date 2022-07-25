#pragma once

#include <libsbp/cpp/state.h>
#include <libserialport.h>
#include <logging/issue_logger.h>
#include <string>

/**
 * @brief Class that implements a Serial Port reader based on the SBP reader
 * interface
 */
class SBPSerialReader : public sbp::IReader {
 public:
  /**
   * @brief Construct a new SBPSerialReader object
   *
   * @param port_name String containing the name of the serial port to use
   * @param connection_string String containing the data needed to open the
   * serial port. The format of the string is: SPEED|DATA BITS|PARITY|STOP
   * BITS|FLOW CONTROL, where: \nSPEED: 9600, or 19200, or 115200, etc.\nDATA
   * BITS: Number of data bits (8 for example)\nPARITY: Parity control (N: none,
   * O: Odd, E: even, M: mark, S: space)\nSTOP BITS: Number of stop bits used
   * (1, 2)\nFLOW CONTROL: (N: none, X: Xon/Xoff, R: RTS/CTS, D: DTR/DSR)
   * Example: 19200|N|8|1|N
   * @param logger Logger facility to use
   * @param read_timeout Timeout in milliseconds for the read operation to
   * start. If 0 (default) the read operation blocks until the requested
   * number of bytes is read or an error occurs
   */
  SBPSerialReader(const std::string& port_name,
                  const std::string& connection_string, const LoggerPtr& logger,
                  const uint32_t read_timeout = 0U) noexcept;

  /**
   * @brief Move Construct a new SBPSerialReader object
   *
   * @param rhs SBPSerialReader to construct from
   */
  SBPSerialReader(SBPSerialReader&& rhs) noexcept;

  /**
   * @brief Destroy the SBPSerialReader object
   */
  virtual ~SBPSerialReader();

  // Deleted methods
  SBPSerialReader() = delete;
  SBPSerialReader(const SBPSerialReader& rhs) = delete;

  /**
   * @brief Method to read data from the serial connection
   *
   * @param buffer Buffer to save the readed data. It must be long enough to
   * contain buffer_length bytes
   * @param buffer_length Max number of bytes to read
   * @return Number of bytes actually readed
   */
  s32 read(u8* buffer, u32 buffer_length) override;

 private:
  /**
   * @brief Method to close the port and release the resources
   */
  void closePort() noexcept;

  /**
   * @brief Method to configure the port
   *
   * @param params Object containing the parameters to set
   * @return true If the setting was OK
   * @return false If the settings failed
   */
  bool setPortSettings(const class SerialParameterSplitter& params) noexcept;

  sp_port* port_;    /** @brief Pointer to a libserialport structure
                        representing a port */
  LoggerPtr logger_; /** @brief Logging facility */
  uint32_t read_timeout_{0U}; /** @brief read timeout in ms */
};
