#include <data_sources/sbp_tcp_datasource.h>
#include <gtest/gtest.h>
#include <iostream>

#include<test/mocked_logger.h>

// This test must be run with the PC connected to a running device through
// Ethernet

constexpr uint16_t DEFAULT_VALID_PORT = 55555;
constexpr uint16_t DEFAULT_INVALID_PORT = 8082;
const std::string DEFAULT_VALID_IP = "127.0.0.1";
const std::string DEFAULT_INVALID_IP = "0.0.0.11";


// *************************************************************************
// TCPReader
TEST(TCPReader, ConnectWithUnexistentIPInvalidPort) {
  auto logger = std::make_shared<MockedLogger>();
  SbpTCPDataSource reader(DEFAULT_INVALID_IP, DEFAULT_INVALID_PORT, logger,
                          100);
  ASSERT_FALSE(reader.isValid());
}

TEST(TCPReader, ConnectWithUnexistentIPValidPort) {
  auto logger = std::make_shared<MockedLogger>();
  SbpTCPDataSource reader(DEFAULT_INVALID_IP, DEFAULT_VALID_PORT, logger, 100);
  ASSERT_FALSE(reader.isValid());
}

TEST(TCPReader, ConnectWithExistentIPValidPort) {
  auto logger = std::make_shared<MockedLogger>();
  SbpTCPDataSource reader(DEFAULT_VALID_IP, DEFAULT_VALID_PORT, logger, 100);
  ASSERT_TRUE(reader.isValid());
}

TEST(TCPReader, ReadPackageWithInvalidReader) {
  auto logger = std::make_shared<MockedLogger>();
  SbpTCPDataSource reader(DEFAULT_INVALID_IP, DEFAULT_VALID_PORT, logger, 100);
  ASSERT_FALSE(reader.isValid());
  ASSERT_EQ(-1, reader.read(nullptr, 100));
}

TEST(TCPReader, ReadPackageWithNullBuffer) {
  auto logger = std::make_shared<MockedLogger>();
  SbpTCPDataSource reader(DEFAULT_VALID_IP, DEFAULT_VALID_PORT, logger, 100);
  ASSERT_TRUE(reader.isValid());
  ASSERT_EQ(-1, reader.read(nullptr, 100));
}

TEST(TCPReader, ReadPackageOK) {
  auto logger = std::make_shared<MockedLogger>();
  SbpTCPDataSource reader(DEFAULT_VALID_IP, DEFAULT_VALID_PORT, logger, 100);
  ASSERT_TRUE(reader.isValid());
  u8 buffer[100];
  const int32_t result = reader.read(buffer, 100);
  ASSERT_TRUE((result > 0) && (result <= 100));
}
