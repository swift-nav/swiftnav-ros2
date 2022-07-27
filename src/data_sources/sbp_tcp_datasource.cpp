#include <data_sources/sbp_tcp_datasource.h>

#include <cstring>

#if defined(__linux__)
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
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

static constexpr uint32_t CONNECT_TIMEOUT = 20;  // twenty seconds

// TODO: Add read timeout
SbpTCPDataSource::SbpTCPDataSource(const std::string& ip, const uint16_t port,
                                   const LoggerPtr& logger,
                                   const uint32_t read_timeout) noexcept
    : logger_(logger), read_timeout_(read_timeout) {
  LOG_DEBUG(logger_,
            "Creating TCP Reader for ip: " << ip << " and port: " << port);
  if (!initSockets()) return;
  openSocket(ip, port);
}

SbpTCPDataSource::SbpTCPDataSource(SbpTCPDataSource&& rhs) noexcept {
  closeSocket();
  socket_id_ = rhs.socket_id_;
  rhs.socket_id_ = -1;
  logger_ = rhs.logger_;
  rhs.logger_.reset();
}

SbpTCPDataSource::~SbpTCPDataSource() {
  closeSocket();
  deinitSockets();
  LOG_DEBUG(logger_, "TCP reader finished");
}

s32 SbpTCPDataSource::read(u8* buffer, u32 buffer_length) {
  if (!buffer) {
    LOG_ERROR(logger_, "Buffer passed to SbpTCPDataSource::read is NULL");
    return -1;
  } else if (!isValid()) {
    LOG_ERROR(logger_,
              "Read operation requested on an uninitialized SbpTCPDataSource");
    return -1;
  }

  struct timeval timeout {
    0, read_timeout_ * 1000
  };
  fd_set read_set;
  FD_ZERO(&read_set);
  FD_SET(socket_id_, &read_set);

  int result = select(socket_id_ + 1, &read_set, nullptr, nullptr, &timeout);
  if (result == -1) {
    LOG_ERROR(logger_,
              "Error: " << GET_SOCKET_ERROR() << " waiting for a read");
    return -1;
  } else if (result == 0) {
    LOG_WARN(logger_, "Timeout waiting to receive data from socket");
    return -1;
  }

  result = recv(socket_id_, buffer, buffer_length, 0);
  if (result > 0) {
    return result;
  } else if (result == 0) {
    LOG_WARN(logger_, "Connection closed by peer");
    return -1;
  } else {
    LOG_ERROR(logger_, "Error (" << GET_SOCKET_ERROR() << ") while reading");
    return result;
  }
}

bool SbpTCPDataSource::initSockets() noexcept {
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

void SbpTCPDataSource::deinitSockets() noexcept {
#if defined(__linux__)

#else
  WSACleanup();
#endif  // __linux__
}

void SbpTCPDataSource::closeSocket() noexcept {
  if (isValid())
#if defined(__linux__)
    close(socket_id_);
  socket_id_ = -1;
#else
    closesocket(socket_id_);
  socket_id_ = INVALID_SOCKET;
#endif  // __linux__
}

bool SbpTCPDataSource::isValid() const noexcept {
#if defined(__linux__)
  return (socket_id_ != -1);
#else
  return (socket_id_ != INVALID_SOCKET);
#endif  // __linux__
}

void SbpTCPDataSource::openSocket(const std::string& ip,
                                  const uint16_t port) noexcept {
  socket_id_ = socket(AF_INET, SOCK_STREAM, 0);
  if (!isValid()) {
    LOG_FATAL(logger_, "socket() failed. (" << GET_SOCKET_ERROR() << ")");
    return;
  }

  if (!setNonBlocking()) {
    LOG_FATAL(logger_, "Can't make the socket non-blocking");
    closeSocket();
    return;
  }

  if (!connectSocket(ip, port)) {
    LOG_FATAL(logger_, "Error: " << GET_SOCKET_ERROR()
                                 << " trying to connect the socket");
    closeSocket();
  } else {
    LOG_INFO(logger_, "Connected!");
  }
}

bool SbpTCPDataSource::setNonBlocking() noexcept {
#if defined(_WIN32)
  if (socket_id_ == INVALID_SOCKET) return false;
  unsigned long mode = 1;
  return (ioctlsocket(fd, FIONBIO, &mode) == 0) ? true : false;
#else
  int flags = fcntl(socket_id_, F_GETFL, 0);
  if (flags == -1) return false;
  flags |= O_NONBLOCK;
  return (fcntl(socket_id_, F_SETFL, flags) == 0) ? true : false;
#endif
}

bool SbpTCPDataSource::connectSocket(const std::string& ip,
                                     const uint16_t port) noexcept {
  struct sockaddr_in server;

  LOG_INFO(logger_, "Connecting...");
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = inet_addr(ip.c_str());
  server.sin_port = htons(port);
  const int result =
      connect(socket_id_, reinterpret_cast<sockaddr*>(&server), sizeof(server));
  if (result == -1) {
#if defined(_WIN32)
    if (WSAGetLastError() != WSAEWOULDBLOCK) return false;
#else
    if (errno != EINPROGRESS) return false;
#endif  // _WIN32

    fd_set connect_set;
    FD_ZERO(&connect_set);
    FD_SET(socket_id_, &connect_set);

    struct timeval timeout {
      CONNECT_TIMEOUT, 0
    };

    switch (select(socket_id_ + 1, nullptr, &connect_set, nullptr, &timeout)) {
      case -1:
      case 0:
        return false;
        break;

      default:
        return (FD_ISSET(socket_id_, &connect_set));
        break;
    }
  } else {
    return true;
  }
}
