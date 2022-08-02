#include <data_sources/sbp_serial_datasource.h>
#include <gtest/gtest.h>
#include <iostream>

// TODO: Make the devices name right
#if defined(_WIN32)
constexpr char VALID_PORT[] = "COM1";
constexpr char INVALID_PORT[] = "COM28";
#else
constexpr char VALID_PORT[] = "/dev/ttyS0";
constexpr char INVALID_PORT[] = "/dev/ttyTAM32";
#endif  // _WIN32

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

// *************************************************************************
// SerialReader
TEST(SerialReader, ConnectWithUnexistentDevice) {
  auto logger = std::make_shared<MockedLogger>();
  SbpSerialDataSource reader(INVALID_PORT, "19200|N|8|1|N", logger, 2000);
  ASSERT_FALSE(reader.isValid());
}

TEST(SerialReader, ConnectWithExistentDeviceButInvalidConnStr) {
  auto logger = std::make_shared<MockedLogger>();
  SbpSerialDataSource reader(VALID_PORT, "19200|T|8|1|W", logger, 2000);
  ASSERT_FALSE(reader.isValid());
}

TEST(SerialReader, ConnectWithExistentDevice) {
  auto logger = std::make_shared<MockedLogger>();
  SbpSerialDataSource reader(VALID_PORT, "19200|N|8|1|N", logger, 2000);
  ASSERT_TRUE(reader.isValid());
}

TEST(SerialReader, ReadPackageWithInvalidReader) {
  auto logger = std::make_shared<MockedLogger>();
  SbpSerialDataSource reader(INVALID_PORT, "19200|N|8|1|N", logger, 2000);
  ASSERT_FALSE(reader.isValid());
  u8 buffer[100];
  ASSERT_EQ(-1, reader.read(buffer, 100));
}

TEST(SerialReader, ReadPackageWithNullBuffer) {
  auto logger = std::make_shared<MockedLogger>();
  SbpSerialDataSource reader(VALID_PORT, "19200|N|8|1|N", logger, 2000);
  ASSERT_TRUE(reader.isValid());
  ASSERT_EQ(-1, reader.read(nullptr, 100));
}

TEST(SerialReader, ReadPackageOK) {
  auto logger = std::make_shared<MockedLogger>();
  SbpSerialDataSource reader(VALID_PORT, "19200|N|8|1|N", logger, 2000);
  ASSERT_TRUE(reader.isValid());
  u8 buffer[100];
  const int32_t result = reader.read(buffer, 100);
  ASSERT_TRUE((result > 0) && (result <= 100));
}
