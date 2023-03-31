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

#include <data_sources/sbp_serial_datasource.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <iostream>

#include<test/mocked_logger.h>

using ::testing::Return;

#if defined(_WIN32)
constexpr char VALID_PORT[] = "COM1";
constexpr char INVALID_PORT[] = "COM28";
#else
constexpr char VALID_PORT[] = "/dev/ttyS0";
constexpr char INVALID_PORT[] = "/dev/ttyTAM32";
#endif  // _WIN32

constexpr char VALID_CONNSTR[] = "115200|N|8|1|N";
constexpr char INVALID_CONNSTR[] = "19200|T|8|1|W";

class MockedSerialPort : public SerialPort {
 public:
  MockedSerialPort(const std::string& device_name,
                   const std::string& connection_string,
                   const uint32_t read_timeout, const uint32_t write_timeout,
                   const LoggerPtr& logger)
      : SerialPort(device_name, connection_string, read_timeout, write_timeout,
                   logger) {}
  MOCK_METHOD(bool, open, (), (noexcept, override));
  MOCK_METHOD(int32_t, read, (uint8_t * buffer, const uint32_t buffer_length),
              (override));
  MOCK_METHOD(int32_t, write,
              (const uint8_t* buffer, const uint32_t buffer_length),
              (override));
  MOCK_METHOD(bool, isValid, (), (const, noexcept, override));
};

// *************************************************************************
// SerialDataSource
TEST(SerialDataSource, ConnectWithUnexistentDevice) {
  auto logger = std::make_shared<MockedLogger>();
  auto mocked_sp = std::make_shared<MockedSerialPort>(
      INVALID_PORT, VALID_CONNSTR, 2000, 2000, logger);
  EXPECT_CALL(*mocked_sp, open).Times(1).WillOnce(Return(false));
  EXPECT_CALL(*mocked_sp, isValid).WillOnce(Return(false));
  SbpSerialDataSource reader(logger, mocked_sp);
  ASSERT_FALSE(reader.isValid());
}

TEST(SerialDataSource, ConnectWithExistentDeviceButInvalidConnStr) {
  auto logger = std::make_shared<MockedLogger>();
  auto mocked_sp = std::make_shared<MockedSerialPort>(
      VALID_PORT, INVALID_CONNSTR, 2000, 2000, logger);
  EXPECT_CALL(*mocked_sp, open).Times(1).WillOnce(Return(false));
  EXPECT_CALL(*mocked_sp, isValid).WillOnce(Return(false));
  SbpSerialDataSource reader(logger, mocked_sp);
  ASSERT_FALSE(reader.isValid());
}

TEST(SerialDataSource, ConnectWithExistentDevice) {
  auto logger = std::make_shared<MockedLogger>();
  auto mocked_sp = std::make_shared<MockedSerialPort>(VALID_PORT, VALID_CONNSTR,
                                                      2000, 2000, logger);
  EXPECT_CALL(*mocked_sp, open).Times(1).WillOnce(Return(true));
  EXPECT_CALL(*mocked_sp, isValid).WillOnce(Return(true));
  SbpSerialDataSource reader(logger, mocked_sp);
  ASSERT_TRUE(reader.isValid());
}

// Reading tests
TEST(SerialDataSource, ReadPackageWithInvalidSource) {
  auto logger = std::make_shared<MockedLogger>();
  std::shared_ptr<SerialPort> serial_port;
  SbpSerialDataSource reader(logger, serial_port);
  ASSERT_FALSE(reader.isValid());
  u8 buffer[100];
  ASSERT_EQ(-1, reader.read(buffer, 100));
}

TEST(SerialDataSource, ReadPackageWithNullBuffer) {
  auto logger = std::make_shared<MockedLogger>();
  auto mocked_sp = std::make_shared<MockedSerialPort>(VALID_PORT, VALID_CONNSTR,
                                                      2000, 2000, logger);
  ON_CALL(*mocked_sp, open).WillByDefault(Return(true));
  EXPECT_CALL(*mocked_sp, read).Times(1).WillOnce(Return(-1));
  ON_CALL(*mocked_sp, isValid).WillByDefault(Return(true));
  SbpSerialDataSource reader(logger, mocked_sp);
  ASSERT_TRUE(reader.isValid());
  ASSERT_EQ(-1, reader.read(nullptr, 100));
}

TEST(SerialDataSource, ReadPackageOK) {
  auto logger = std::make_shared<MockedLogger>();
  auto mocked_sp = std::make_shared<MockedSerialPort>(VALID_PORT, VALID_CONNSTR,
                                                      2000, 2000, logger);
  ON_CALL(*mocked_sp, open).WillByDefault(Return(true));
  EXPECT_CALL(*mocked_sp, read).Times(1).WillOnce(Return(100));
  ON_CALL(*mocked_sp, isValid).WillByDefault(Return(true));
  SbpSerialDataSource reader(logger, mocked_sp);
  ASSERT_TRUE(reader.isValid());
  u8 buffer[100];
  const int32_t result = reader.read(buffer, 100);
  ASSERT_TRUE((result > 0) && (result <= 100));
}

// Writing tests
TEST(SerialDataSource, WritePackageWithInvalidSource) {
  auto logger = std::make_shared<MockedLogger>();
  std::shared_ptr<SerialPort> serial_port;
  SbpSerialDataSource writer(logger, serial_port);
  ASSERT_FALSE(writer.isValid());
  u8 buffer[100];
  ASSERT_EQ(-1, writer.write(buffer, 100));
}

TEST(SerialDataSource, WritePackageWithNullBuffer) {
  auto logger = std::make_shared<MockedLogger>();
  auto mocked_sp = std::make_shared<MockedSerialPort>(VALID_PORT, VALID_CONNSTR,
                                                      2000, 2000, logger);
  ON_CALL(*mocked_sp, open).WillByDefault(Return(true));
  EXPECT_CALL(*mocked_sp, write).Times(1).WillOnce(Return(-1));
  ON_CALL(*mocked_sp, isValid).WillByDefault(Return(true));
  SbpSerialDataSource writer(logger, mocked_sp);
  ASSERT_TRUE(writer.isValid());
  ASSERT_EQ(-1, writer.write(nullptr, 100));
}

TEST(SerialDataSource, WritePackageOK) {
  auto logger = std::make_shared<MockedLogger>();
  auto mocked_sp = std::make_shared<MockedSerialPort>(VALID_PORT, VALID_CONNSTR,
                                                      2000, 2000, logger);
  ON_CALL(*mocked_sp, open).WillByDefault(Return(true));
  EXPECT_CALL(*mocked_sp, write).Times(1).WillOnce(Return(100));
  ON_CALL(*mocked_sp, isValid).WillByDefault(Return(true));
  SbpSerialDataSource writer(logger, mocked_sp);
  ASSERT_TRUE(writer.isValid());
  u8 buffer[100];
  const int32_t result = writer.write(buffer, 100);
  ASSERT_TRUE((result > 0) && (result <= 100));
}
