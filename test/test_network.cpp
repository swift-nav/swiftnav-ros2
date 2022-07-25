#include <gtest/gtest.h>
#include <readers/sbp_tcpreader.h>
#include <iostream>

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
// TCPReader
TEST(TCPReader, ConnectWithUnexistentIP) {
  auto logger = std::make_shared<MockedLogger>();
  SBPTCPReader reader("0.0.0.11", 8082, logger, 100);
  ASSERT_FALSE(reader.isValid());
}

TEST(TCPReader, ConnectWithExistentIP) {
  auto logger = std::make_shared<MockedLogger>();
  SBPTCPReader reader("127.0.0.1", 8082, logger, 100);
  ASSERT_TRUE(reader.isValid());
}

TEST(TCPReader, ReadPackageWithInvalidReader) {
  auto logger = std::make_shared<MockedLogger>();
  SBPTCPReader reader("0.0.0.1", 8082, logger, 100);
  ASSERT_FALSE(reader.isValid());
  ASSERT_EQ(-1, reader.read(nullptr, 100));
}

TEST(TCPReader, ReadPackageWithNullBuffer) {
  auto logger = std::make_shared<MockedLogger>();
  SBPTCPReader reader("127.0.0.1", 8082, logger, 100);
  ASSERT_TRUE(reader.isValid());
  ASSERT_EQ(-1, reader.read(nullptr, 100));
}

TEST(TCPReader, ReadPackageOK) {
  auto logger = std::make_shared<MockedLogger>();
  SBPTCPReader reader("127.0.0.1", 8082, logger, 100);
  ASSERT_TRUE(reader.isValid());
  u8 buffer[100];
  const int32_t result = reader.read(buffer, 100);
  ASSERT_TRUE((result > 0) && (result <= 100));
}

// *************************************************************************
// UDPReader

// *******************************************
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
