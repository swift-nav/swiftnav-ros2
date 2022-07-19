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
   * @param connection_string String containing the data needed to open the
   * serial port. The format of the string is: SPEED|DATA BITS|PARITY|STOP
   * BITS|FLOW CONTROL, where: SPEED: 9600, or 19200, or 115200, etc. DATA BITS:
   * Number of data bits (8 for example) PARITY: Parity control (N: none, O:
   * Odd, E: even, M: mark, S: space) STOP BITS: Number of stop bits used (1, 2)
   * Example: 19200|N|8|1|N
   * @param logger Logger facility to use
   */
  SBPSerialReader(const std::string& connection_string,
                  const LoggerPtr& logger) noexcept;

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
  void closePort() noexcept;

  sp_port* port_;    /** @brief Pointer to a libserialport structure
                        representing a port */
  LoggerPtr logger_; /** @brief Logging facility */
};
