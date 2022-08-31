#include <data_sources/sbp_serial_datasource.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <iostream>

using ::testing::Return;

// TODO: Make the devices name right
#if defined(_WIN32)
constexpr char VALID_PORT[] = "COM1";
constexpr char INVALID_PORT[] = "COM28";
#else
constexpr char VALID_PORT[] = "/dev/ttyS0";
constexpr char INVALID_PORT[] = "/dev/ttyTAM32";
#endif  // _WIN32

constexpr char VALID_CONNSTR[] = "115200|N|8|1|N";
constexpr char INVALID_CONNSTR[] = "19200|T|8|1|W";

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

class MockedSerialPort : public SerialPort {
 public:
  MockedSerialPort(const std::string& device_name,
                   const std::string& connection_string,
                   const uint32_t read_timeout, const uint32_t write_timeout,
                   const LoggerPtr& logger)
      : SerialPort(device_name, connection_string, read_timeout, write_timeout,
                   logger) {}
  MOCK_METHOD(bool, open, (), (noexcept));
  MOCK_METHOD(int32_t, read, (uint8_t * buffer, const uint32_t buffer_length),
              (override));
  MOCK_METHOD(int32_t, write,
              (const uint8_t* buffer, const uint32_t buffer_length),
              (override));
  MOCK_METHOD(bool, isValid, (), (const, noexcept, override));
};

// *************************************************************************
// SerialReader
TEST(SerialReader, ConnectWithUnexistentDevice) {
  auto logger = std::make_shared<MockedLogger>();
  auto mocked_sp =
      new MockedSerialPort(INVALID_PORT, VALID_CONNSTR, 2000, 2000, logger);
  auto serial_port = std::unique_ptr<SerialPort>(mocked_sp);
  EXPECT_CALL(*mocked_sp, open).Times(1).WillOnce(Return(false));
  SbpSerialDataSource reader(logger, serial_port);
  ASSERT_FALSE(reader.isValid());
}

TEST(SerialReader, ConnectWithExistentDeviceButInvalidConnStr) {
  auto logger = std::make_shared<MockedLogger>();
  auto mocked_sp =
      new MockedSerialPort(VALID_PORT, INVALID_CONNSTR, 2000, 2000, logger);
  auto serial_port = std::unique_ptr<SerialPort>(mocked_sp);
  EXPECT_CALL(*mocked_sp, open).Times(1).WillOnce(Return(false));
  SbpSerialDataSource reader(logger, serial_port);
  ASSERT_FALSE(reader.isValid());
}

TEST(SerialReader, ConnectWithExistentDevice) {
  auto logger = std::make_shared<MockedLogger>();
  auto mocked_sp =
      new MockedSerialPort(VALID_PORT, VALID_CONNSTR, 2000, 2000, logger);
  auto serial_port = std::unique_ptr<SerialPort>(mocked_sp);
  EXPECT_CALL(*mocked_sp, open).Times(1).WillOnce(Return(true));
  SbpSerialDataSource reader(logger, serial_port);
  ASSERT_TRUE(reader.isValid());
}

TEST(SerialReader, ReadPackageWithInvalidReader) {
  auto logger = std::make_shared<MockedLogger>();
  auto serial_port = std::unique_ptr<SerialPort>(nullptr);
  SbpSerialDataSource reader(logger, serial_port);
  ASSERT_FALSE(reader.isValid());
  u8 buffer[100];
  ASSERT_EQ(-1, reader.read(buffer, 100));
}

TEST(SerialReader, ReadPackageWithNullBuffer) {
  auto logger = std::make_shared<MockedLogger>();
  auto mocked_sp =
      new MockedSerialPort(VALID_PORT, VALID_CONNSTR, 2000, 2000, logger);
  auto serial_port = std::unique_ptr<SerialPort>(mocked_sp);
  ON_CALL(*mocked_sp, open).WillByDefault(Return(true));
  SbpSerialDataSource reader(logger, serial_port);
  ASSERT_TRUE(reader.isValid());
  EXPECT_CALL(*mocked_sp, read).Times(1).WillOnce(Return(-1));
  ASSERT_EQ(-1, reader.read(nullptr, 100));
}

TEST(SerialReader, ReadPackageOK) {
  auto logger = std::make_shared<MockedLogger>();
  auto mocked_sp =
      new MockedSerialPort(VALID_PORT, VALID_CONNSTR, 2000, 2000, logger);
  auto serial_port = std::unique_ptr<SerialPort>(mocked_sp);
  ON_CALL(*mocked_sp, open).WillByDefault(Return(true));
  SbpSerialDataSource reader(logger, serial_port);
  ASSERT_TRUE(reader.isValid());
  u8 buffer[100];
  EXPECT_CALL(*mocked_sp, read).Times(1).WillOnce(Return(100));
  const int32_t result = reader.read(buffer, 100);
  ASSERT_TRUE((result > 0) && (result <= 100));
}

// TODO: Add Writing tests