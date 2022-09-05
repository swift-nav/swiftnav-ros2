#pragma once

#include <logging/issue_logger.h>
#include <cstdint>
#include <memory>
#include <string>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#endif  // _WIN32

class TCP {
 public:
  TCP() = delete;

  TCP(const std::string& ip, const uint16_t port, const LoggerPtr& logger,
      const uint32_t read_timeout, const uint32_t write_timeout);

  virtual ~TCP();

  virtual bool open() noexcept;

  virtual int32_t read(uint8_t* buffer, const uint32_t buffer_size);

  virtual int32_t write(const uint8_t* buffer, const uint32_t buffer_size);

  virtual bool isValid() const noexcept;

 protected:
  /**
   * @brief Method to init the socket system.
   *
   * @return String containig the error. Empty if OK
   */
  std::string initSockets() noexcept;

  /**
   * @brief Method to clean and free socket system resources
   */
  void deinitSockets() noexcept;

  /**
   * @brief Closes the open socket in use
   */
  void closeSocket() noexcept;

  /**
   * @brief Method that opens and connects the socket
   */
  bool openSocket() noexcept;

  /**
   * @brief Sets the socket to be non-blocking
   *
   * @return true The socket could be configured
   * @return false The configuration failed
   */
  bool setNonBlocking() noexcept;

  /**
   * @brief Method to connect the socket client to the server
   *
   * @return true The socket is connected
   * @return false The socket failed to connect
   */
  bool connectSocket() noexcept;

#if defined(_WIN32)
  SOCKET socket_id_{INVALID_SOCKET}; /** @brief Windows Socket */
#else
  int socket_id_{-1}; /** @brief Linux socket */
#endif  // _WIN32

  LoggerPtr logger_;           /** @brief Logging facility */
  std::string ip_;             /** @brief IP to connect to */
  uint16_t port_;              /** @brief TCP port to connect to */
  uint32_t read_timeout_{0U};  /** @brief Read timeout in ms */
  uint32_t write_timeout_{0U}; /** @brief Write timeout in ms */
};