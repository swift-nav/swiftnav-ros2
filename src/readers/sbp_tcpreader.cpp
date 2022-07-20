#include <readers/sbp_tcpreader.h>

#include <cstring>

#if defined(__linux__)
#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define GET_SOCKET_ERROR() (errno)
#else
#pragma comment(lib, "ws2_32.lib")

#define GET_SOCKET_ERROR() WSAGetLastError()

#endif  // __linux__

// TODO: Add read timeout
SBPTCPReader::SBPTCPReader(const std::string& ip, const uint16_t port,
                           const LoggerPtr& logger) noexcept
    : logger_(logger) {
  LOG_DEBUG(logger_,
            "Creating TCP Reader for ip: " << ip << " and port: " << port);
  if (!initSockets()) return;
  openSocket(ip, port);
}

SBPTCPReader::SBPTCPReader(SBPTCPReader&& rhs) noexcept {
  closeSocket();
  socket_id_ = rhs.socket_id_;
  rhs.socket_id_ = -1;
  logger_ = rhs.logger_;
  rhs.logger_.reset();
}

SBPTCPReader::~SBPTCPReader() {
  closeSocket();
  deinitSockets();
  LOG_DEBUG(logger_, "TCP reader finished");
}

s32 SBPTCPReader::read(u8* buffer, u32 buffer_length) {
  if (!buffer) {
    LOG_ERROR(logger_, "Buffer passed to SBPTCPReader::read is NULL");
    return -1;
  } else if (!isValid()) {
    LOG_ERROR(logger_,
              "Read operation requested on an uninitialized SBPTCPReader");
    return -1;
  }

  const s32 result = recv(socket_id_, buffer, buffer_length, 0);
  if (result > 0) {
  } else if (result == 0) {
    LOG_WARN(logger_, "Connection closed by peer");
  } else {
    LOG_ERROR(logger_, "Error (" << GET_SOCKET_ERROR() << ") while reading");
  }

  return result;
}

bool SBPTCPReader::initSockets() noexcept {
#if defined(__linux__)
  return true;
#else
  WSADATA d;
  if (WSAStartup(MAKEWORD(2, 2), &d)) {
    LOG_FATAL(logger_, "Failed to initialize sockets");
    return false;
  } else {
    return true;
  }
#endif  // __linux__
}

void SBPTCPReader::deinitSockets() noexcept {
#if defined(__linux__)

#else
  WSACleanup();
#endif  // __linux__
}

void SBPTCPReader::closeSocket() noexcept {
  if (isValid())
#if defined(__linux__)
    close(socket_id_);
#else
    closesocket(socket_id_);
#endif  // __linux__
}

bool SBPTCPReader::isValid() const noexcept {
#if defined(__linux__)
  return (socket_id_ != -1);
#else
  return (socket_id_ != INVALID_SOCKET);
#endif  // __linux__
}

void SBPTCPReader::openSocket(const std::string& ip,
                              const uint16_t port) noexcept {
  struct addrinfo hints;
  struct addrinfo* peer_address;

  memset(&hints, 0, sizeof(hints));
  hints.ai_socktype = SOCK_STREAM;
  std::string str_port = std::to_string(port);
  if (getaddrinfo(ip.c_str(), str_port.c_str(), &hints, &peer_address)) {
    LOG_FATAL(logger_, "getaddrinfo() failed. (" << GET_SOCKET_ERROR() << ")");
    return;
  }

  socket_id_ = socket(peer_address->ai_family, peer_address->ai_socktype,
                      peer_address->ai_protocol);
  if (!isValid()) {
    LOG_FATAL(logger_, "socket() failed. (" << GET_SOCKET_ERROR() << ")");
    return;
  }

  LOG_INFO(logger_, "Connecting...");
  if (connect(socket_id_, peer_address->ai_addr, peer_address->ai_addrlen)) {
    LOG_FATAL(logger_, "connect() failed. (" << GET_SOCKET_ERROR() << ")");
    closeSocket();
  } else {
    LOG_INFO(logger_, "Connected!");
  }

  freeaddrinfo(peer_address);
}
