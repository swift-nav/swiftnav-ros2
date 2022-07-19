#pragma once

#include <libsbp/cpp/state.h>
#include <logging/issue_logger.h>
#include <cstdint>
#include <string>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#endif  // _WIN32

/**
 * @brief Class that implements a TCP reader based on the SBP reader interface
 * (IReader)
 */
class SBPTCPReader : public sbp::IReader {
 public:
  /**
   * @brief Construct a new SBPTCPReader object
   *
   * @param ip IP address to connect to. It could be in IPV4 or IPV6 format
   * @param port TCP port to connect to
   * @param logger Logger facility to use
   */
  SBPTCPReader(const std::string& ip, const uint16_t port,
               const LoggerPtr& logger) noexcept;

  /**
   * @brief Move Construct a new SBPTCPReader object
   *
   * @param rhs SBPTCPReader to construct from
   */
  SBPTCPReader(SBPTCPReader&& rhs) noexcept;

  /**
   * @brief Destroy the SBPTCPReader object
   */
  virtual ~SBPTCPReader();

  // Deleted methods
  SBPTCPReader() = delete;
  SBPTCPReader(const SBPTCPReader& rhs) = delete;

  /**
   * @brief Method to read data from the TCP connection
   *
   * @param buffer Buffer to save the readed data. It must be long enough to
   * contain buffer_length bytes
   * @param buffer_length Max number of bytes to read
   * @return Number of bytes actually readed
   */
  s32 read(u8* buffer, u32 buffer_length) override;

 private:
  /**
   * @brief Method to init the socket system.
   *
   * @return true Sockets ready for use
   * @return false Sockets not reaady for use
   */
  bool initSockets() noexcept;

  /**
   * @brief Method to clean and free socket system resources
   */
  void deinitSockets() noexcept;

  /**
   * @brief Closes the open socket in use
   */
  void closeSocket() noexcept;

  /**
   * @brief Method to determine if the internal socket is valid or not
   *
   * @return true Socket is valid
   * @return false Socket is not valid
   */
  bool isValid() const noexcept;

  /**
   * @brief Method that opens and connects the socket
   *
   * @param ip IP to connect to
   * @param port Port to connect to
   */
  void openSocket(const std::string& ip, const uint16_t port) noexcept;

#if defined(_WIN32)
  SOCKET socket_id_{INVALID_SOCKET}; /** @brief Windows Socket */
#else
  int socket_id_{-1}; /** @brief Linux socket */
#endif  // _WIN32

  LoggerPtr logger_; /** @brief Logging facility */
};
