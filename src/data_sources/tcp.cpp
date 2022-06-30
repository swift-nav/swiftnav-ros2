/*
 * Copyright (C) 2015-2023 Swift Navigation Inc.
 * Contact: https://support.swiftnav.com
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <data_sources/tcp.h>

#if defined(__linux__)
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <string>
#include <cstring>


#define GET_SOCKET_ERROR() (errno)
#else
#pragma comment(lib, "ws2_32.lib")

#define GET_SOCKET_ERROR() WSAGetLastError()

#endif  // __linux__

static constexpr uint32_t CONNECT_TIMEOUT = 20;  // [s]
static constexpr uint32_t MS_TO_US = 1000;
static constexpr uint32_t S_TO_MS = 1000;

TCP::TCP(const std::string& ip, const uint16_t port, const LoggerPtr& logger,
         const uint32_t read_timeout, const uint32_t write_timeout)
    : logger_(logger),
      ip_(ip),
      port_(port),
      read_timeout_(read_timeout),
      write_timeout_(write_timeout) {}

TCP::~TCP() {
  closeSocket();
  deinitSockets();
}

bool TCP::open() noexcept {
  const std::string error = initSockets();
  if (!error.empty()) {
    LOG_FATAL(logger_, error.c_str());
    return false;
  } else {
    return openSocket();
  }
}

void TCP::close() noexcept { 
  closeSocket(); 
}

int32_t TCP::read(uint8_t* buffer, const uint32_t buffer_size) {
  struct timeval timeout {
    (read_timeout_ / S_TO_MS),
    (read_timeout_ % S_TO_MS) * MS_TO_US
  };

  fd_set read_set;
  FD_ZERO(&read_set);
  FD_SET(socket_id_, &read_set);

  int result = select(socket_id_ + 1, &read_set, nullptr, nullptr, &timeout);
  if (-1 == result) {
    LOG_ERROR(logger_, "Waiting for data error (%u)",
              GET_SOCKET_ERROR());
    return -1;
  } else if (0 == result) {
    LOG_ERROR(logger_, "Receiving data timeout");
    return -1;
  }

  result = recv(socket_id_, buffer, buffer_size, 0);
  if (result >= 0) {
    return result;
  } else {
    LOG_ERROR(logger_, "Receiving data error (%u)", GET_SOCKET_ERROR());
    return result;
  }
}

int32_t TCP::write(const uint8_t* buffer, const uint32_t buffer_size) {
  struct timeval timeout {
    (write_timeout_ / S_TO_MS),
    (write_timeout_ % S_TO_MS) * MS_TO_US
  };

  fd_set write_set;
  FD_ZERO(&write_set);
  FD_SET(socket_id_, &write_set);

  int result = select(socket_id_ + 1, nullptr, &write_set, nullptr, &timeout);
  if (result == -1) {
    LOG_ERROR(logger_,
              "Error: %u waiting for the socket to be ready to write data",
              GET_SOCKET_ERROR());
    return -1;
  } else if (result == 0) {
    LOG_WARN(logger_,
             "Timeout waiting for the socket to be ready to write data");
    return -1;
  }

  result = send(socket_id_, buffer, buffer_size, 0);
  if (result > 0) {
    return result;
  } else {
    LOG_ERROR(logger_, "Error (%u) while writing", GET_SOCKET_ERROR());
    return result;
  }
}

std::string TCP::initSockets() noexcept {
#if defined(__linux__)
  return {};
#else
  WSADATA d;
  if (WSAStartup(MAKEWORD(2, 2), &d))
    return std::string("Failed to initialize sockets");
  else
    return {};
#endif  // __linux__
}

void TCP::deinitSockets() noexcept {
#if defined(__linux__)

#else
  WSACleanup();
#endif  // __linux__
}

void TCP::closeSocket() noexcept {
#if defined(__linux__)
  if (socket_id_ != -1) ::close(socket_id_);
  socket_id_ = -1;
#else
  if (socket_id_ != INVALID_SOCKET) closesocket(socket_id_);
  socket_id_ = INVALID_SOCKET;
#endif  // __linux__
}

bool TCP::isValid() const noexcept {
#if defined(__linux__)
  return (socket_id_ != -1);
#else
  return (socket_id_ != INVALID_SOCKET);
#endif  // __linux__
}

bool TCP::openSocket() noexcept {
  socket_id_ = socket(AF_INET, SOCK_STREAM, 0);
  if (!isValid()) {
    LOG_FATAL(logger_, "socket() failed. (%u)", GET_SOCKET_ERROR());
    return false;
  }

  if (!setNonBlocking()) {
    LOG_FATAL(logger_, "Can't make the socket non-blocking");
    return false;
  }

  return connectSocket();
}

bool TCP::setNonBlocking() noexcept {
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

std::string ipFromAddress(const std::string& addr) {
  if (addr.empty())
    return {};

  addrinfo hints;
  addrinfo* result = nullptr;
  addrinfo* ptr;

  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;
  if (getaddrinfo(addr.c_str(), nullptr, &hints, &result) == 0)
  {
    sockaddr_in* sockaddr_ipv4;

    for (ptr = result; ptr != nullptr; ptr = ptr->ai_next) {
      if (ptr->ai_family == AF_INET) {
        char Addr[30];

        sockaddr_ipv4 = reinterpret_cast<sockaddr_in*>(ptr->ai_addr);
        return std::string(inet_ntop(AF_INET, &sockaddr_ipv4->sin_addr, Addr, sizeof(Addr)));
      }
    }
  }
    
  return {};
}

bool TCP::connectSocket() noexcept {
  struct sockaddr_in server;

  LOG_INFO(logger_, "Connecting to %s:%u",ip_.c_str(),port_);
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = inet_addr( ipFromAddress(ip_).c_str() );
  server.sin_port = htons(port_);
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
        LOG_INFO(logger_, "Connected");
        return (FD_ISSET(socket_id_, &connect_set));
        break;
    }
  } else {
    return true;
  }
}
