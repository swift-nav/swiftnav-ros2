#include <data_sources/sbp_tcp_datasource.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <iostream>

using ::testing::Return;

constexpr uint16_t DEFAULT_VALID_PORT = 55555;
constexpr uint16_t DEFAULT_INVALID_PORT = 8082;
const std::string DEFAULT_VALID_IP = "127.0.0.1";
const std::string DEFAULT_INVALID_IP = "0.0.0.11";

// *************************************************************************
// Dummy console implementation of a Logger
class MockedLogger : public IIssueLogger {
 public:
  void logDebug(const std::stringstream& ss) override {
    std::cout << "DEBUG->" << ss.str() << std::endl;
  }
  void logInfo(const std::stringstream& ss) override {
    std::cout << "INFO->" << ss.str() << std::endl;
  }
  void logWarning(const std::stringstream& ss) override {
    std::cout << "WARN->" << ss.str() << std::endl;
  }
  void logError(const std::stringstream& ss) override {
    std::cout << "ERROR->" << ss.str() << std::endl;
  }
  void logFatal(const std::stringstream& ss) override {
    std::cout << "FATAL->" << ss.str() << std::endl;
  }
};

class MockedTCP : public TCP {
 public:
  MockedTCP(const std::string& ip, const uint16_t port, const LoggerPtr& logger,
            const uint32_t read_timeout, const uint32_t write_timeout)
      : TCP(ip, port, logger, read_timeout, write_timeout) {}
  MOCK_METHOD(bool, open, (), (noexcept));
  MOCK_METHOD(int32_t, read, (uint8_t * buffer, const uint32_t buffer_length),
              (override));
  MOCK_METHOD(int32_t, write,
              (const uint8_t* buffer, const uint32_t buffer_length),
              (override));
  MOCK_METHOD(bool, isValid, (), (const, noexcept, override));
};

// *************************************************************************
// TCPDataSource
TEST(TCPDataSource, TestInvalidConnection) {
  auto logger = std::make_shared<MockedLogger>();
  auto mocked_tcp = std::make_unique<MockedTCP>(
      DEFAULT_INVALID_IP, DEFAULT_INVALID_PORT, logger, 2000, 2000);
  EXPECT_CALL(*mocked_tcp, open).Times(1).WillOnce(Return(false));
  SbpTCPDataSource reader(logger, std::move(mocked_tcp));
  ASSERT_FALSE(reader.isValid());
}

TEST(TCPDataSource, TestValidConnection) {
  auto logger = std::make_shared<MockedLogger>();
  auto mocked_tcp = std::make_unique<MockedTCP>(
      DEFAULT_VALID_IP, DEFAULT_VALID_PORT, logger, 2000, 2000);
  EXPECT_CALL(*mocked_tcp, open).Times(1).WillOnce(Return(true));
  ON_CALL(*mocked_tcp, isValid).WillByDefault(Return(true));
  SbpTCPDataSource reader(logger, std::move(mocked_tcp));
  ASSERT_TRUE(reader.isValid());
}

// Reading tests
TEST(TCPDataSource, TestReadingWithInvalidObject) {
  auto logger = std::make_shared<MockedLogger>();
  std::unique_ptr<TCP> tcp;
  SbpTCPDataSource reader(logger, std::move(tcp));
  ASSERT_FALSE(reader.isValid());
  uint8_t buffer[100];
  ASSERT_EQ(-1, reader.read(buffer, 100));
}

TEST(TCPDataSource, TestReadingWithNullBuffer) {
  auto logger = std::make_shared<MockedLogger>();
  auto mocked_tcp = std::make_unique<MockedTCP>(
      DEFAULT_VALID_IP, DEFAULT_VALID_PORT, logger, 2000, 2000);
  EXPECT_CALL(*mocked_tcp, open).Times(1).WillOnce(Return(true));
  EXPECT_CALL(*mocked_tcp, read).Times(0);
  ON_CALL(*mocked_tcp, isValid).WillByDefault(Return(true));
  SbpTCPDataSource reader(logger, std::move(mocked_tcp));
  ASSERT_TRUE(reader.isValid());
  ASSERT_EQ(-1, reader.read(nullptr, 100));
}

TEST(TCPDataSource, TestReadPackageOK) {
  auto logger = std::make_shared<MockedLogger>();
  auto mocked_tcp = std::make_unique<MockedTCP>(
      DEFAULT_VALID_IP, DEFAULT_VALID_PORT, logger, 2000, 2000);
  EXPECT_CALL(*mocked_tcp, open).Times(1).WillOnce(Return(true));
  EXPECT_CALL(*mocked_tcp, read).Times(1).WillOnce(Return(100));
  ON_CALL(*mocked_tcp, isValid).WillByDefault(Return(true));
  SbpTCPDataSource reader(logger, std::move(mocked_tcp));
  ASSERT_TRUE(reader.isValid());
  uint8_t buffer[100];
  const int32_t result = reader.read(buffer, 100);
  ASSERT_TRUE((result > 0) && (result <= 100));
}

// Writing tests
TEST(TCPDataSource, TestWritingWithInvalidObject) {
  auto logger = std::make_shared<MockedLogger>();
  std::unique_ptr<TCP> tcp;
  SbpTCPDataSource writer(logger, std::move(tcp));
  ASSERT_FALSE(writer.isValid());
  uint8_t buffer[100];
  ASSERT_EQ(-1, writer.write(buffer, 100));
}

TEST(TCPDataSource, TestWritingWithNullBuffer) {
  auto logger = std::make_shared<MockedLogger>();
  auto mocked_tcp = std::make_unique<MockedTCP>(
      DEFAULT_VALID_IP, DEFAULT_VALID_PORT, logger, 2000, 2000);
  EXPECT_CALL(*mocked_tcp, open).Times(1).WillOnce(Return(true));
  EXPECT_CALL(*mocked_tcp, write).Times(0);
  ON_CALL(*mocked_tcp, isValid).WillByDefault(Return(true));
  SbpTCPDataSource writer(logger, std::move(mocked_tcp));
  ASSERT_TRUE(writer.isValid());
  ASSERT_EQ(-1, writer.write(nullptr, 100));
}

TEST(TCPDataSource, TestWritePackageOK) {
  auto logger = std::make_shared<MockedLogger>();
  auto mocked_tcp = std::make_unique<MockedTCP>(
      DEFAULT_VALID_IP, DEFAULT_VALID_PORT, logger, 2000, 2000);
  EXPECT_CALL(*mocked_tcp, open).Times(1).WillOnce(Return(true));
  EXPECT_CALL(*mocked_tcp, write).Times(1).WillOnce(Return(100));
  ON_CALL(*mocked_tcp, isValid).WillByDefault(Return(true));
  SbpTCPDataSource writer(logger, std::move(mocked_tcp));
  ASSERT_TRUE(writer.isValid());
  uint8_t buffer[100];
  const int32_t result = writer.write(buffer, 100);
  ASSERT_TRUE((result > 0) && (result <= 100));
}
