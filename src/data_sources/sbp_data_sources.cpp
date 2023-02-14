/*
 * Copyright (C) 2010-2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <data_sources/sbp_data_sources.h>

std::shared_ptr<SbpFileDataSource> dataSourceFactory(
    const std::string& sbp_file_path, const LoggerPtr& logger) {
  return std::make_shared<SbpFileDataSource>(sbp_file_path, logger);
}

std::shared_ptr<SbpSerialDataSource> dataSourceFactory(
    const std::string& device_name, const std::string& connection_str,
    const uint32_t read_timeout, const uint32_t write_timeout,
    const LoggerPtr& logger) {
  auto serial_port = std::make_unique<SerialPort>(
      device_name, connection_str, read_timeout, write_timeout, logger);
  return std::make_shared<SbpSerialDataSource>(logger, std::move(serial_port));
}

std::shared_ptr<SbpTCPDataSource> dataSourceFactory(
    const std::string& host_ip, const uint16_t host_port,
    const uint32_t read_timeout, const uint32_t write_timeout,
    const LoggerPtr& logger) {
  auto tcp = std::make_unique<TCP>(host_ip, host_port, logger, read_timeout,
                                   write_timeout);
  return std::make_shared<SbpTCPDataSource>(logger, std::move(tcp));
}
